#include "trtSettings.h"
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>

#include "trtUart.h"
#include "trtkernel_1284.c"
#include "trtUART.c"

/********************************************************************/
//						UART Functions
/********************************************************************/
FILE uart0 = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

/********************************************************************/
//						FSM Variables
/********************************************************************/
// two semaphores to protect message --
// sender must wait until message is received 
// -- init to 1 becuase don't need to wait for first message
// receiver must wait until message is sent
#define SEM_TX_WAIT  4
#define SEM_RX_WAIT  5
// semaphore to protect shared variable
#define SEM_SHARED 6

// input arguments to each thread
// not actually used in this example
int args[2] ;

// binary states
#define FALSE 0
#define TRUE 1

// switch states
unsigned char mode;
#define TEST 0
#define NORMAL 1
unsigned char function; 
#define fLOCK 0
#define fKEY 1
#define fBOTH 2
unsigned char authorization;
#define AUTHORIZED '>'
#define UNAUTHORIZED '<'

// states for the FSMs
#define IDLE 0
#define REQUEST 1
#define CHALLENGE 2
#define RESPONSE 3

#define LOCK 0
#define UNLOCK 1
//state of the lock (unlocked or locked_)
unsigned char state;
//state variable for the IR_key task
unsigned char key_state;
//state variable for the IR_lock task
unsigned char lock_state;
//task to transmit
unsigned char lock_command;
//recieved time used to prevent man in the middle attacks
char* challenge_time_string; 
/********************************************************************/
// 					IR Loopback Variables
/********************************************************************/
/* UART baud rate */
// for IR link
#define IR_UART_BAUD  4800
#define PC_UART_BAUD  9600

// ISR
#define SUSPEND cli();
#define RESUME sei();

void initialize(void); //all the usual mcu stuff 

//timeout counter  
// time to give up and assume you will never get a response   
#define ir_rx_timeout 100 //milliseconds
#define ir_tx_timeout 100
   
// task timers and global time counters (mSec)
volatile unsigned long tx_send_time, rx_rec_time, time;

//************************
// string buffers and ISR comm buffers
#define buffer_size 64
volatile char ir_tx_buffer[buffer_size], ir_rx_buffer[buffer_size] ;
// count for filling(sending) buffers
volatile char ir_tx_count, ir_rx_count ;
// read flags
volatile char ir_tx_ready, ir_rx_ready ;
// start and}characters for packet
#define start_token '#'
#define end_token '%'

//************************
// bit twiddling macros
#define READ(U, N) (((U) >> (N)) & 1u)
#define SET(U, N) ((void)((U) |= 1u << (N)))
#define CLR(U, N) ((void)((U) &= ~(1u << (N))))
#define FLIP(U, N) ((void)((U) ^= 1u << (N)))

/********************************************************************/
// 							IR Functions
/********************************************************************/
//timer 0 overflow ISR --
// increments task timers
// Full duplex transmit/receive
// sends ir character (if ready)
// receives ir char (if valid)
// builds ir receive buffer
ISR (TIMER0_COMPA_vect) {
	unsigned char c ;
	    
	//**********************
  	// send an ir char if tx is ready and still char in buffer to send
	// and USART is ready
	if (ir_tx_ready ){ //&& ir_tx_buffer[ir_tx_count]>0
		if (UCSR1A & (1<<UDRE1)) UDR1 = ir_tx_buffer[ir_tx_count++];
		if (ir_tx_buffer[ir_tx_count]==0x00) ir_tx_ready = 0 ; //}of buffer
		if (ir_tx_count >= buffer_size) ir_tx_ready = 0; // buffer overrun
	}
	
	//**********************
  	// recv an ir char if data ready 
  	// otherwise set c to null 
	if (UCSR1A & (1<<RXC1) ) {
		c = UDR1 ; // valid char 
	}
	else c = 0 ; // nonvalid

	//**********************
	// append character to the received string
	// if character is valid and we expect a string
	if (c>0) { //&& (ir_rx_ready==0)) {

		if (c == start_token) { // restart the string
			ir_rx_count = 0 ;
		}

		else if (c == end_token){ //}the string
			ir_rx_buffer[ir_rx_count] = 0x00 ;
			ir_rx_ready = 1 ;
		}

		else { // add to string and check for buffer overrun
			ir_rx_buffer[ir_rx_count++] = c ;
			if (ir_rx_count >= buffer_size) { // buffer overrun
				ir_rx_ready = 2;
				ir_rx_buffer[buffer_size-1] = 0x00 ;
				ir_rx_count = buffer_size -1 ; //???
			}
		}

	} //}if c>0
} 

//**********************************************************
// IR send
// Input transmitter id and string packet payload
void ir_send_packet(char tx_id, char ir_data[])
{
	char ir_tx_ck_sum, ir_tx_ck_sum1, ir_tx_ck_sum2 ;
	char i ;
	
	// compute check sum on input data string only
	ir_tx_ck_sum = 0;
	for (i=0; i<strlen(ir_data); i++)
		ir_tx_ck_sum ^= ir_data[i] ;
	// chop the check sum into two 1/2 bytes and add a bit
	ir_tx_ck_sum1 = 0x10 + (ir_tx_ck_sum & 0x0f) ;
	ir_tx_ck_sum2 = 0x10 + ((ir_tx_ck_sum>>4) & 0x0f) ;

	// format send string
  	sprintf(ir_tx_buffer,"%c%c%s%c%c%c", 
		start_token, tx_id, ir_data, ir_tx_ck_sum1, ir_tx_ck_sum2, end_token );
  	ir_tx_count = 0 ;
  	ir_tx_ready = 1 ;

	tx_send_time =  trtCurrentTime();

	// wait 
	while (ir_tx_ready && (trtCurrentTime() < tx_send_time + ir_tx_timeout)) {};
	
}

//**********************************************************
// IR Receive
// input expected transmitter id
// returns zero if payload is valid and returns payload
// 1 means no data; 2 means buffer overrun; 3 means bad tx id; 4 means bad checksum
char ir_rec_packet(char tx_id, char ir_data[])
{
	char rx_status = 0;
	char ir_rx_ck_sum, ir_rx_ck_sum_ref;
	char i ;
	char buf_len ;

	buf_len = (char)strlen(ir_rx_buffer) ;
	if (buf_len >= buffer_size) buf_len = buffer_size-1;

	// error check
	if (ir_rx_ready == 2) {rx_status = 2; return rx_status;} // buffer overrun
	if (ir_rx_ready == 0) {rx_status = 1; return rx_status;} // no data -- timeout
	if (ir_rx_buffer[0] != tx_id) {rx_status = 3; return rx_status;} // bad transmitter id
	if (ir_rx_ready != 1) {rx_status = 1; return rx_status;} // invalid data

	// compute receive checksum
	ir_rx_ck_sum = 0 ;
	for (i=1; i<buf_len-2; i++)
		ir_rx_ck_sum ^= ir_rx_buffer[i] ;
	
	ir_rx_ck_sum_ref = (ir_rx_buffer[buf_len-2] & 0x0f) +
		 (ir_rx_buffer[buf_len-1]<<4) ;

    if (ir_rx_ck_sum_ref != ir_rx_ck_sum) 
		{rx_status = 4; return rx_status;}  // bad check sum

	// set up the valid data return stuff
	ir_rx_buffer[0] = ' '; // strip the transmit id
	ir_rx_buffer[buf_len-2] = 0x00; // strip the check sum and trailer
	ir_rx_ready = 0 ;
	//trim initial space using ir_rx_buffer+1
	strlcpy(ir_data, ir_rx_buffer+1, buffer_size) ; 

	return rx_status ;
}

//********************************************************** 
//Set it all up
void initialize(void)
{
  DDRC = 0x01;    	// led connections
  PORTC = 0x00;
  DDRB = 0x00; 		// switch connections
  PORTB = 0xff; 	// pullup on
  //********************    
  //set up timer 0 for 1 mSec timebase 
  TIMSK0= (1<<OCIE0A);	//turn on timer 0 cmp match ISR 
  OCR0A = 249 ;//
  //set prescalar to divide by 64 (4 microsec/tick)
  TCCR0B= 3; //;	
  // turn on clear-on-match
  TCCR0A= (1<<WGM01) ;
  
  //******************** 

  // IR carrier generator
  // set up timer 2 for square wave with NO ISR
  // 56000 Hz => 1/2 cycle 8.928 microsec = 143 cycles
  // Connect D7 thru resistor to (+)IR_LED(-) to TXD:: D7--/\/\--|>|----TXD
  //
  OCR2A = 142 ; //143 cycles/half-period
  //  count at full rate
  TCCR2B =  1;	
  // set to toggle OC2A, clear on match,
  TCCR2A = (1<<COM2A0) | (1<<WGM21) ;
  // PORT D.7 is OC2A
  DDRD = (1<<PIND7) ;	
  //********************
  //init UART1 for IR comm
  UBRR1L = (F_CPU / (16UL * IR_UART_BAUD)) - 1;
  UCSR1B = _BV(TXEN1) | _BV(RXEN1); /* tx/rx enable */
  UCSR1C = (1<<UCSZ11) | (1<<USBS1) ; // 7 bit | 2 stop bits

  //init UART0 for PC comm
  UBRR0L = (F_CPU / (16UL * PC_UART_BAUD)) - 1;
  UCSR0B = _BV(TXEN0) ; //| _BV(RXEN1); /* tx/rx enable */
  fprintf(&uart0,"\n\r...Starting IR comm ...\n\r");
  
  //********************
  //crank up the ISRs
  sei();
}  
//==================================================

/********************************************************************/
// 							FSM Tasks
/********************************************************************/
// Synchronized task to read the switches
void read_switches() {
	trtWait(SEM_SHARED);
	// Read the switches & decide to set the lock on or off
	// Read switch 0 to select test or normal mode
	if (~PINB & 0x01) {
		mode = NORMAL;
		// Read switch 1 to select key or lock functionality
		if (~PINB & 0x02) function = fKEY;
		else function = fLOCK;
	}
	else {
		mode = TEST;
		function = fBOTH;
	}
		// Read switch 2 to select authorized or unauthorized
	authorization = ~PINB & 0x04 ? AUTHORIZED : UNAUTHORIZED;
	fprintf(stdout, "%c %c %c", mode, function, authorization);
	trtSignal(SEM_SHARED);
}

// --- define task 1  ----------------------------------------
void IR_key(void* args) {
	uint32_t rel, dead;
	//char ir_tx_data[buffer_size];
	//char ir_rx_data[buffer_size];
	// Initialize the state variable
	//key_state = IDLE;
	// Read the switches
	//read_switches();
	// FSM for IR_key
	while (TRUE) {
		//fprintf(stdout, "Made it to IR_key task!");
		/*
		// Check that the function is either KEY or BOTH
		if ((function == fKEY) | (function == fBOTH )) {
			switch (key_state) {
				case IDLE:
					// Check to see if a button is pressed
					// 0x08 is to lock & 0x10 is to unlock
					if ((~PINB & 0x08) | (~PINB & 0x10)) {
						key_state = REQUEST;
						lock_command = ~PINB & 0x08 ? LOCK : UNLOCK;
					}
					break;
				case REQUEST:
					sprintf(ir_tx_data, "r");
					trtWait(SEM_TX_WAIT);
					ir_send_packet('K', ir_tx_data);
					trtSignal(SEM_TX_WAIT);
					key_state = CHALLENGE;
					break;
				case CHALLENGE:
					trtWait(SEM_RX_WAIT);
					char rec_state = ir_rec_packet('L', ir_rx_data);
					trtSignal(SEM_RX_WAIT);
					// Transmission correctly recieved. 
					if (rec_state == 0) {
						challenge_time_string = ir_rx_data;
						state = RESPONSE;
					}
					break;
				case RESPONSE:
					sprintf(ir_tx_data, "%c%c%s", lock_command, authorization, challenge_time_string);
					trtWait(SEM_TX_WAIT);
					ir_send_packet('K', ir_tx_data);
					trtSignal(SEM_TX_WAIT);
					key_state = IDLE;
				default:
					// Something is wrong. Recover to IDLE. 
					key_state = IDLE;
			}
		}
		*/
		// Sleep
	    rel = trtCurrentTime() + SECONDS2TICKS(0.1);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.2);
	    trtSleepUntil(rel, dead);
	}
}

// --- define task 2  ----------------------------------------
void IR_lock(void* args) {
	uint32_t rel, dead;
	initialize();
	//char ir_tx_data[buffer_size];
	//char ir_rx_data[buffer_size];
	//char rec_state;
	// Initialize the state variable
	//lock_state = IDLE;
	// Initialize the state of the lock
	//state = LOCK;
	// Read the switches 
	//read_switches();
	// FSM for IR_key
	while (TRUE) {
		//fprintf(stdout, "Made it to IR_lock task!");
		/*
		// Check that the function is either KEY or BOTH
		if ((function == fLOCK) | (function == fBOTH)) {
			switch (lock_state) {
				case IDLE:
					trtWait(SEM_RX_WAIT);
					rec_state = ir_rec_packet('K', ir_rx_data);
					trtSignal(SEM_RX_WAIT);
					if (rec_state == 0) {
						state = CHALLENGE;
					}
					break;
				case CHALLENGE:
					sprintf(ir_tx_data, "%ld", trtCurrentTime());
					trtWait(SEM_TX_WAIT);
					ir_send_packet('L', ir_tx_data);
					trtSignal(SEM_TX_WAIT);
					state = RESPONSE;
					break;
				case RESPONSE:
					trtWait(SEM_RX_WAIT);
					rec_state = ir_rec_packet('K', ir_rx_data);
					trtSignal(SEM_RX_WAIT);
					// Transmission correctly recieved. 
					if (rec_state == 0) {
						char op = ir_rx_data[0];
						char auth = ir_rx_data[1];
						char* data = ir_rx_data+2;
						if ((trtCurrentTime() - atol(data) <= 1000) & (auth == '>')) {
							// Change LED state. Unlocked = light. Locked = no light
							if (((state == LOCK) & (op == UNLOCK)) | ((state == UNLOCK) & (op == LOCK))) {
								state = (op == UNLOCK)? UNLOCK : LOCK;
								PORTC = PORTC ^ 0x01 ;
							}
						}
						op = (op == LOCK) ? 'L' : 'U';
						auth = (auth == AUTHORIZED) ? 'Y' : 'N';
						trtWait(SEM_TX_WAIT) ; // wait for the message to be received
						fprintf(stdout, op + ' ' + auth + ' ' + data + '\n' + '\r');
						trtSignal(SEM_RX_WAIT) ; // tell receiver messsage is ready
						state = IDLE;
					}
					break;
				default:
					// Something is wrong. Recover to IDLE. 
					key_state = IDLE;
			}
		}
		*/
		// Sleep
	    rel = trtCurrentTime() + SECONDS2TICKS(0.1);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.2);
	    trtSleepUntil(rel, dead);
	}
}

// --- Main Program ----------------------------------
int main(void) {
  //init the UART -- trt_uart_init() is in trtUart.c
  trt_uart_init();
  stdout = stdin = stderr = &uart0;
  fprintf(stdout,"\n\r TRT 9feb2009\n\r\n\r");
    // start TRT
  trtInitKernel(80); // 80 bytes for the idle task stack

  // --- create semaphores ----------
  // You must create the first two semaphores if you use the uart
  trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; // uart receive ISR semaphore
  trtCreateSemaphore(SEM_STRING_DONE,0) ;  // user typed <enter>

  // message protection
  trtCreateSemaphore(SEM_TX_WAIT, 1) ; // message send interlock
  trtCreateSemaphore(SEM_RX_WAIT, 0) ; // message receive interlock
  
  // variable protection
  trtCreateSemaphore(SEM_SHARED, 1) ; // protect shared variable

  // --- create tasks  ----------------
  trtCreateTask(IR_lock, 2000, SECONDS2TICKS(0.1), SECONDS2TICKS(0.2), &(args[0]));
  //trtCreateTask(IR_key, 1000, SECONDS2TICKS(0.1), SECONDS2TICKS(0.2), &(args[1]));

  // --- Idle task --------------------------------------
  // For debugging, blink an LED
  // For production, you would probably comment out the body of the WHILE
  while (1) {
  	PORTC = PORTC ^ 0x01 ;
  	_delay_ms(500) ;	
  }

} // main
