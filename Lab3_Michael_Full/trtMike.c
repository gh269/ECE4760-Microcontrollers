// test TRT with two led blinking tasks
// and two UART tasks

#include "trtSettings.h"
#include "trtkernel_1284.c"
#include <util/delay.h>
#include <stdio.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

// serial communication library
// Don't mess with the semaphores
#define SEM_RX_ISR_SIGNAL 1
#define SEM_STRING_DONE 2 // user hit <enter>
#include "trtUart.h"
#include "trtUart.c"

// UART file descriptor
// putchar and getchar are in uart.c
//FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// allow task2 to control task1
#define SEM_TASK1_WAIT 3

// two semaphores to protect message --
// sender must wait until message is received 
// -- init to 1 becuase don't need to wait for first message
// receiver must wait until message is sent
#define SEM_SEND_IR  4
#define SEM_RECV_IR  5

// semaphore to protect shared variable
#define SEM_SHARED 7

// the usual
#define begin {
#define end }

/* CPU frequency */
#define F_CPU 16000000UL
/* UART baud rate */
// for IR link
#define IR_UART_BAUD  4800
#define PC_UART_BAUD  9600

// ISR
#define SUSPEND cli();
#define RESUME sei();
 
// State variables and defines
uint8_t state;

#define IDLE 0
#define CHALLENGE 1
#define REQUEST 2
#define RESPONSE 3

// Configuration switch states and defines
uint8_t test_mode;		// TEST or NORMAL
uint8_t key_or_lock;	// KEY or BASE_LOCK (in normal)
						// LOOP (in test)
uint8_t authorized;		// AUTHORIZED or UNAUTHORIZED
uint8_t lock_command;   // Controlled by pushbutton
uint8_t unlock_command; // Controlled by pushbutton

#define TEST 0
#define NORMAL 1
#define BASE_LOCK 0
#define KEY 1
#define LOOP 2
#define AUTHORIZED 1
#define UNAUTHORIZED 0

// Global Variables
char key_id;	// < or >
uint32_t challenge_time_string;

// Lock and key transmit ID characters
char key_tx_id = 'K';
char lock_rx_id = 'L';

// IR RX and TX functions
void send_task(void);
void key_send_request(void);
void lock_send_challenge( void );
void key_send_confirm( uint32_t challenge_time_string );
void ir_send_packet(char tx_id, char ir_data[]);

void recv_task(void);
char key_recv_challenge(char key_tx_id, char ir_rx_data[]);
char lock_recv_task(char key_tx_id, char ir_rx_data[]);
char ir_rec_packet(char tx_id, char ir_data[]);

void recv_any_task(void);
char ir_rec_any_packet(char ir_data[]);

void initialize(void); //all the usual mcu stuff 

// TIMER STUFF
// time to give up and assume you will never get a response   
#define ir_rx_timeout 100 //milliseconds
#define ir_tx_timeout 100
// task timers and global time counters (mSec)
volatile unsigned long tx_send_time, rx_rec_time;
uint32_t task_count; 
uint32_t rel, dead;
int error_count ;

//************************
// string buffers and ISR comm buffers
#define buffer_size 64
volatile char ir_tx_buffer[buffer_size], ir_rx_buffer[buffer_size] ;
// count for filling(sending) buffers
volatile char ir_tx_count, ir_rx_count ;
// read flags
volatile char ir_tx_ready, ir_rx_ready ;
// start and end characters for packet
#define start_token '#'
#define end_token '%'

//************************
// bit twiddling macros
#define READ(U, N) (((U) >> (N)) & 1u)
#define SET(U, N) ((void)((U) |= 1u << (N)))
#define CLR(U, N) ((void)((U) &= ~(1u << (N))))
#define FLIP(U, N) ((void)((U) ^= 1u << (N)))

// --- Blink LEDs and run uart ---------------------------------
// input arguments to each thread
// not actually used in this example
int args[4] ;

// a value passed between task3 and task4
uint16_t Message_vin ;

// shared variable to count number of total task invocations
//uint32_t task_count ;


//**********************************************************
//timer 0 overflow ISR --
// increments task timers
// Full duplex transmit/receive
// sends ir character (if ready)
// receives ir char (if valid)
// builds ir receive buffer
ISR (TIMER0_COMPA_vect) 
{
	unsigned char c ;
	
	//**********************
  	// send an ir char if tx is ready and still char in buffer to send
	// and USART is ready
	if (ir_tx_ready ){ //&& ir_tx_buffer[ir_tx_count]>0
		if (UCSR1A & (1<<UDRE1)) UDR1 = ir_tx_buffer[ir_tx_count++];
		if (ir_tx_buffer[ir_tx_count]==0x00) ir_tx_ready = 0 ; // end of buffer
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

		else if (c == end_token){ // end the string
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

	}
}

//*********************************************************
// UART 0 interface to use with PC
// --- putchar ---
// uses uart 0
void putchar_uart0(char c)
{
  //if (c == '\n')
  //  uart_putchar('\r', stream);
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
}

// -- getchar ---
// users uart 0 -- very basic input
char getchar_uart0(void)
{
  uint8_t c;
  loop_until_bit_is_set(UCSR0A, RXC0);
  return c = UDR0;
}

FILE uart0 = FDEV_SETUP_STREAM(putchar_uart0, getchar_uart0, _FDEV_SETUP_RW);


// --- Tasks for printing IR data using TRT ---------
void led1(void* args) 
begin
  	initialize();

	while(1)
	begin
		// wait on semaphore SEM_TASK1_WAIT 
		// which is signaled from task 2
		trtWait(SEM_SEND_IR) ; 
		// blink the led
		send_task();
		trtSleepUntil(trtCurrentTime() + SECONDS2TICKS(.2), trtCurrentTime() + SECONDS2TICKS(.2));
		trtSignal(SEM_RECV_IR);
		
		// update task count
		//trtWait(SEM_SHARED) ;
		//task_count++;
		//trtSignal(SEM_SHARED);
	end
end

void led2(void* args) 
  begin
	while(1)
	begin
		trtWait(SEM_RECV_IR) ; 
		recv_any_task();
		trtSleepUntil(trtCurrentTime() + SECONDS2TICKS(.2), trtCurrentTime() + SECONDS2TICKS(.2));
		trtSignal(SEM_SEND_IR);
		// update task count
		//trtWait(SEM_SHARED) ;
		//task_count++;
		//trtSignal(SEM_SHARED);
	end
end

void IR_key(void* args) 
{
	while(1)
	{
        if (key_or_lock == KEY) 
	    {
		    if (lock_command) {state = REQUEST;}

			if (state == REQUEST) {
		    	trtWait(SEM_SEND_IR) ; 
				key_send_request();
				trtSignal(SEM_SEND_IR);
				state = CHALLENGE;
			}
			else if (state == CHALLENGE) {
				char ir_rx_data[buffer_size];			    
				char rx_status;

				trtWait(SEM_RECV_IR);
				rx_status = key_recv_challenge(key_tx_id, ir_rx_data);
				trtSignal(SEM_RECV_IR);
				if (!rx_status) {
					challenge_time_string = trtCurrentTime();
					state = RESPONSE;
				}
			}
			else if (state == RESPONSE) {
				trtWait(SEM_SEND_IR);
				key_send_confirm(challenge_time_string);
				trtSignal(SEM_SEND_IR);
				state = IDLE;
			}
			trtWait(SEM_SHARED) ;
			task_count++;
			trtSignal(SEM_SHARED);

			// Sleep
	    	rel = trtCurrentTime() + SECONDS2TICKS(0.09);
	    	dead = trtCurrentTime() + SECONDS2TICKS(0.1);
	    	trtSleepUntil(rel, dead);
		}
	 }  // end while
}

void IR_lock(void* args)  {
	state = IDLE;
	while(1) {
		if (key_or_lock == BASE_LOCK) {
			if (state == IDLE) {
				char rx_status;
				char ir_rx_data[buffer_size]; // to store received payload
				trtWait(SEM_RECV_IR);
				rx_status = lock_recv_task(key_tx_id, ir_rx_data); //receive a packet from the key. ir_rx_data should be modified by this.
				trtSignal(SEM_RECV_IR);
				if (rx_status && (ir_rx_data[0] == "r")) {
					state = CHALLENGE;
				}
			}
			else if (state == CHALLENGE) { //base-lock sends challenge to key
				trtWait(SEM_SEND_IR);
				lock_send_challenge(); // ok to trtWait before sprintf?
				trtSignal(SEM_SEND_IR);
				state = RESPONSE;
			}
			else if (state == RESPONSE) {
				char ir_rx_data[buffer_size];
				uint32_t challenge_time = 0;
				char key_id = 0;
				char rx_status = 0;
				
				trtWait(SEM_RECV_IR);
				rx_status = lock_recv_task(key_tx_id, ir_rx_data); // to store received payload
				trtSignal(SEM_RECV_IR);
				// parse ir_rx_data and checks the key ID ("<" or ">") and time
				fscanf(stdin, "%c %ld", key_id, challenge_time, &ir_rx_data);
				if (!rx_status) {
					if (key_id == AUTHORIZED && (trtCurrentTime - challenge_time > TICKSPERSECOND)) {
						PORTD |= 0x04; //unlock
					}
				}
			else { state == IDLE; }
			}
		}
	}
}




//************************************************
// IR Send and Receive Functions
//************************************************

void send_task(void) 
begin  
	char tx_id = 'T'; // note: this is a quoted char
	char ir_tx_data[buffer_size] ;

	// transmit
	sprintf(ir_tx_data,"%ld, %d, %d", trtCurrentTime(), task_count, error_count); //
	// send payload from tx_id
	ir_send_packet(tx_id, ir_tx_data);
end  


// key sends 'r'
void key_send_request( void ) 
begin  
	char tx_id = 'K';
	char ir_tx_data[buffer_size] ;
	sprintf(ir_tx_data,"%c", 'r'); //
	ir_send_packet(tx_id, ir_tx_data);
end

// Lock sends challenge time back to key
void lock_send_challenge( void ) 
begin  
	char tx_id = 'L';
	char ir_tx_data[buffer_size] ;

	// transmit valid
	if (lock_command) {
		sprintf(ir_tx_data, "%c>%s", lock_command, challenge_time_string);
	}
	
	// send payload from tx_id
	ir_send_packet(tx_id, ir_tx_data);
end 

// after receiving challenge the key sends confirmation to lock
// with the challenge time
void key_send_confirm(uint32_t challenge_time_string) 
begin  
	char ir_tx_data[buffer_size];
	char tx_id = 'K'; // note: this is a quoted char
	
	if (authorized == AUTHORIZED) {
		sprintf(ir_tx_data, "%c>%s", lock_command, challenge_time_string);
	}
	else {
		sprintf(ir_tx_data, "%c<%s", lock_command, challenge_time_string);
	}
	// send payload from tx_id
	ir_send_packet(tx_id, ir_tx_data);
end


//**********************************************************
// IR send
// Input transmitter id and string packet payload
void ir_send_packet(char tx_id, char ir_data[])
begin
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

	tx_send_time = trtCurrentTime() ;
	// wait 
	while (ir_tx_ready && (trtCurrentTime() < tx_send_time + ir_tx_timeout)) {};
	
end


//**********************************************************          
// receive it back
void recv_task(void) 
begin  
	char rx_id = 'T';  // note: this is a quoted char
	char rx_status = 0 ;
	char ir_rx_data[buffer_size]  ;
	
	// receive
	rx_status = ir_rec_packet(rx_id, ir_rx_data) ;
	if (!rx_status) {
		//ir_rx_data[6] = 0;
		fprintf(&uart0, "PAYLOAD=%s\n\r", ir_rx_data);
	}
	else {
		fprintf(&uart0, "%d%s\n\r", rx_status, ir_rx_data);
		error_count++ ;
	}
	task_count++;
end

//**********************************************************
// Returns rx_status and stores payload in ir_rx_data (passed in) if valid
char key_recv_challenge( char rx_id, char ir_rx_data[]) 
begin
	//char rx_id = 'K';  // Key identifier K sends "r" to base-lock
	char rx_status = 0 ;
	
	rx_status = ir_rec_packet(rx_id, ir_rx_data) ;
	if (!rx_status) {
		//ir_rx_data[6] = 0;
		fprintf(&uart0, "PAYLOAD=%ld\n\r", ir_rx_data);
	}
	else {
		fprintf(&uart0, "%d%s\n\r", rx_status, ir_rx_data);
	}
	return rx_status;
end


//**********************************************************
// Returns rx_status and stores payload in ir_rx_data (passed in) if valid
char lock_recv_task( char rx_id, char ir_rx_data[]) 
begin
	//char rx_id = 'K';  // Key identifier K sends "r" to base-lock
	char rx_status = 0 ;
	
	// receive
	rx_status = ir_rec_packet(rx_id, ir_rx_data) ;
	if (!rx_status) {
		//ir_rx_data[6] = 0;
		fprintf(&uart0, "PAYLOAD=%s\n\r", ir_rx_data);
	}
	else {
		fprintf(&uart0, "%d%s\n\r", rx_status, ir_rx_data);
	}
	return rx_status;
end

//**********************************************************
// IR Receive
// input expected transmitter id
// returns zero if payload is valid and returns payload
// 1 means no data; 2 means buffer overrun; 3 means bad tx id; 4 means bad checksum
char ir_rec_packet(char tx_id, char ir_data[])
begin
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
end



//**********************************************************          
// receive it back
void recv_any_task(void) 
begin  
//	char rx_id = '1';  // note: this is a quoted char
	char rx_status = 0 ;
	char ir_rx_data[buffer_size]  ;
	
	// receive
	rx_status = ir_rec_any_packet(ir_rx_data) ;
	if (rx_status>=0x30) {
		//ir_rx_data[6] = 0;
		fprintf(&uart0, "TX_ID=%c\n\r", rx_status); // transmitter number
		fprintf(&uart0, "PAYLOAD=%s\n\r", ir_rx_data);// payload
	}
	else {
		fprintf(&uart0, "%d%s\n\r", rx_status, ir_rx_data);
		error_count++ ;
	}
	task_count++;
end

//**********************************************************
// IR Receive ANY transmitter
// 
// returns tx_id (>0x30) if payload is valid and returns payload
// return==1 means no data; 2 means buffer overrun; 4 means bad checksum
char ir_rec_any_packet(char ir_data[])
begin
	char rx_status = 0;
	char ir_rx_ck_sum, ir_rx_ck_sum_ref;
	char i ;
	char buf_len ;

	buf_len = (char)strlen(ir_rx_buffer) ;
	if (buf_len >= buffer_size) buf_len = buffer_size-1;

	// error check
	if (ir_rx_ready == 2) return 2 ; // buffer overrun
	if (ir_rx_ready == 0) return 1 ; // no data -- timeout
	if (ir_rx_ready != 1) {rx_status = 1; return rx_status;} // invalid data
	// if (ir_rx_buffer[0] != tx_id) return 3 ; // bad transmitter id

	// compute receive checksum
	ir_rx_ck_sum = 0 ;
	for (i=1; i<buf_len-2; i++)
		ir_rx_ck_sum ^= ir_rx_buffer[i] ;
	
	ir_rx_ck_sum_ref = (ir_rx_buffer[buf_len-2] & 0x0f) +
		 (ir_rx_buffer[buf_len-1]<<4) ;

	if (ir_rx_ck_sum_ref != ir_rx_ck_sum) return 4 ; // bad check sum

	// set up the valid return stuff
	// if no errors, then return transmitter ID
	if (rx_status == 0) rx_status = ir_rx_buffer[0] ;
	ir_rx_buffer[0] = ' '; // strip the transmit id from string
	ir_rx_buffer[buf_len-2] = 0x00; // strip the check sum and trailer
	ir_rx_ready = 0 ;
	//trim initial space using ir_rx_buffer+1
	strlcpy(ir_data, ir_rx_buffer+1, buffer_size) ;

	return rx_status ;
end
	

//********************************************************** 
//Set it all up
void initialize(void)
begin
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
  
  // Debugging
  lock_command = 1;

  //********************
  //crank up the ISRs
  sei();

end  
//==================================================

   
// --- Main Program ----------------------------------
int main(void) {

  DDRC = 0xff;    // led connections
  PORTC = 0xff;
  DDRB = 0x00; // switch connections
  PORTB = 0xff; // pullup on

  //init the UART -- trt_uart_init() is in trtUart.c
  stdout = stdin = stderr = &uart0;//&uart_str;
  fprintf(stdout,"\n\r TRT 09feb09\n\r\n\r");
  //fprintf(&uart0, "it works?");
  trt_uart_init(); 

  // start TRT
  trtInitKernel(80); // 80 bytes for the idle task stack

  // --- create semaphores ----------
  // You must creat the first two semaphores if you use the uart
  //trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; // uart receive ISR semaphore
  //trtCreateSemaphore(SEM_STRING_DONE,0) ;  // user typed <enter>
  
  // Task synch
  trtCreateSemaphore(SEM_TASK1_WAIT, 0) ; // task2 controls task1 rate
  
  // message protection
  trtCreateSemaphore(SEM_SEND_IR, 1) ; // message send interlock
  trtCreateSemaphore(SEM_RECV_IR, 0) ; // message receive interlock
  
  // variable protection
  trtCreateSemaphore(SEM_SHARED, 1) ; // protect shared variable

 // --- creat tasks  ----------------
  //trtCreateTask(led1, 1000, SECONDS2TICKS(0.1), SECONDS2TICKS(0.2), &(args[0]));
  //trtCreateTask(led2, 100, SECONDS2TICKS(0.1), SECONDS2TICKS(0.2), &(args[1]));
  trtCreateTask(IR_key, 1000, SECONDS2TICKS(0.1), SECONDS2TICKS(0.2), &(args[0]));
  //trtCreateTask(IR_lock, 1000, SECONDS2TICKS(0.1), SECONDS2TICKS(0.2), &(args[0]));

  // --- Idle task --------------------------------------
  // For debugging, blink an LED
  // For production, you would probably comment out the body of the WHILE
  while (1) {}
} 
