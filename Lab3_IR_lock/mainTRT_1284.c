#include "trtSettings.h"
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h> 

#include "trtUart.h"
#include "trtkernel_1284.c"
#include "trtUART_usart_1.c"
#include "IR_comm_loopback_GCC1284_v2.c"

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
	//trtWait(SEM_UART);
	//fprintf(stdout, "Made it to IR_key task!");
	//trtSignal(SEM_UART);
	uint32_t rel, dead;
	//char ir_tx_data[buffer_size];
	//char ir_rx_data[buffer_size];
	// Initialize the state variable
	//key_state = IDLE;
	// Read the switches
	//read_switches();
	// FSM for IR_key
	while (TRUE) {
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
	//trtWait(SEM_UART);
	fprintf(&uart_str, "Made it to IR_lock task!");
	//trtSignal(SEM_UART);
	uint32_t rel, dead;
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

  DDRC = 0x01;    	// led connections
  PORTC = 0x00;
  DDRB = 0x00; 		// switch connections
  PORTB = 0xff; 	// pullup on

  //init the UART -- trt_uart_init() is in trtUart.c
  trt_uart_init();
  stdout = stdin = stderr = &uart_str;
  fprintf(stdout,"\n\r TRT 9feb2009\n\r\n\r");

  initialize_IR();

  // --- create semaphores ----------
  // You must creat the first two semaphores if you use the uart
  trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; // uart receive ISR semaphore
  trtCreateSemaphore(SEM_STRING_DONE,0) ;  // user typed <enter>
  trtCreateSemaphore(SEM_UART, 1) ;

  // message protection
  trtCreateSemaphore(SEM_TX_WAIT, 1) ; // message send interlock
  trtCreateSemaphore(SEM_RX_WAIT, 0) ; // message receive interlock
  
  // variable protection
  trtCreateSemaphore(SEM_SHARED, 1) ; // protect shared variable

  // --- create tasks  ----------------
  trtCreateTask(IR_lock, 1000, SECONDS2TICKS(0.1), SECONDS2TICKS(0.2), &(args[0]));
  trtCreateTask(IR_key, 1000, SECONDS2TICKS(0.1), SECONDS2TICKS(0.2), &(args[1]));

  // --- Idle task --------------------------------------
  // For debugging, blink an LED
  // For production, you would probably comment out the body of the WHILE
  while (1) {
  	PORTC = PORTC ^ 0x01 ;
	_delay_ms(500) ;
	
  }

} // main
