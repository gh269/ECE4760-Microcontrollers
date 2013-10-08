#include "trtSettings.h"
#include "trtkernel_1284.c"
#include "IR_comm_loopback_GCC1284_v2.c"
#include <util/delay.h>
#include <stdio.h>

// serial communication library
// Don't mess with the semaphores
#define SEM_RX_ISR_SIGNAL 1
#define SEM_STRING_DONE 2 // user hit <enter>
#include "trtUart.h"
#include "trtUart_usart_1.c"
// UART file descriptor
// putchar and getchar are in uart.c
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// allow task2 to control task1
#define SEM_TASK1_WAIT 3

// two semaphores to protect message --
// sender must wait until message is received 
// -- init to 1 becuase don't need to wait for first message
// receiver must wait until message is sent
#define SEM_TX_WAIT  4
#define SEM_RX_WAIT  5

// semaphore to protect shared variable
#define SEM_SHARED 7

// the usual
#define begin {
#define end }

// --- Blink LEDs and run uart ---------------------------------
// input arguments to each thread
// not actually used in this example
int args[4] ;

// a value passed between task3 and task4
uint16_t Message_vin ;

// shared variable to count number of total task invocations
uint32_t task_count ;

// switches
#define STATESW 0x01
#define FUNCTIONSW 0x02
#define AUTHSW 0x04
#define LOCKSW 0x08
#define UNLOCKSW 0x10

// switch states
unsigned char mode;
#define TEST 0
#define NORMAL 1
unsigned char function; 
#define LOCK 0
#define KEY 1
#define BOTH 2
unsigned char authorization;
#define AUTHORIZED '>'
#define UNAUTHORIZED '<'

// states for the FSMs
#define IDLE 0
#define REQUEST 1
#define CHALLENGE 2
#define RESPONSE 3

// binary states
#define FALSE 0
#define TRUE 1

//state variable for the IR_key task
unsigned char key_state;
//state variable for the IR_lock task
unsigned char lock_state;
//task to transmit
unsigned char lock_command;
#define LOCK_SIG 0
#define UNLOCK_SIG 1

//recieved time used to prevent man in the middle attacks
volatile char challenge_time_string[buffer_size]; 

// Synchronized task to read the switches
void read_switches() {
	trtWait(SEM_SHARED);
	// Read the switches & decide to set the lock on or off
	// Read switch 0 to select test or normal mode
	if (~PINB & 0x01) {
		mode = NORMAL;
		// Read switch 1 to select key or lock functionality
		if (~PINB & 0x02) function = KEY;
		else function = LOCK;
	}
	else {
		mode = TEST;
		function = BOTH;
	}
		// Read switch 2 to select authorized or unauthorized
	authorization = ~PINB & 0x04 ? AUTHORIZED : UNAUTHORIZED;
	trtSignal(SEM_SHARED);
}

// --- define task 1  ----------------------------------------
void IR_key(void* args) {
	uint32_t rel, dead;
	char ir_tx_data[buffer_size];
	char ir_rx_data[buffer_size];
	// Initialize the state variable
	key_state = IDLE;
	// Read the switches
	read_switches();
	// FSM for IR_key
	while (TRUE) {
		// Check that the function is either KEY or BOTH
		if (function == KEY | function == BOTH ) {
			switch (key_state) {
				case IDLE:
					// Check to see if a button is pressed
					// 0x08 is to unlock & 0x10 is to lock
					if (~PINB & 0x08 | ~PINB & 0x10) {
						key_state = REQUEST;
						lock_command = ~PINB & 0x08 ? UNLOCK_SIG : LOCK_SIG;
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
				case DEFAULT:
					// Something is wrong. Recover to IDLE. 
					key_state = IDLE;
			}
		}
		// Sleep
	    rel = trtCurrentTime() + SECONDS2TICKS(0.1);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.1);
	    trtSleepUntil(rel, dead);
	}
}

// --- define task 2  ----------------------------------------
void IR_lock(void* args) {
	uint32_t rel, dead;
	char ir_tx_data[buffer_size];
	char ir_rx_data[buffer_size];
	// Initialize the state variable
	lock_state = IDLE;
	// Read the switches 
	read_switches();
	// FSM for IR_key
	while (TRUE) {
		// Check that the function is either KEY or BOTH
		if (function == LOCK) {
			switch (lock_state) {
				case IDLE:
					trtWait(SEM_RX_WAIT);
					char rec_state = ir_rec_packet('K', ir_rx_data);
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
					char rec_state = ir_rec_packet('K', ir_rx_data);
					trtSignal(SEM_RX_WAIT);
					// Transmission correctly recieved. 
					if (rec_state == 0) {
						char op = ir_rx_data[0];
						char auth = ir_rx_data[1];
						char[] data = ir_rx_data+2;
						if (trtCurrentTime() - atol(data) <= 1000 & auth == '>') {
							// DO SOMETHING HERE!
							trtWait(SEM_TX_WAIT) ; // wait for the message to be received
							Message_vin = vin ;
							trtSignal(SEM_RX_WAIT) ; // tell receiver messsage is ready
						};
						state = IDLE;
					}
					break;
				case DEFAULT:
					// Something is wrong. Recover to IDLE. 
					key_state = IDLE;
			}
		}
		// Sleep
	    rel = trtCurrentTime() + SECONDS2TICKS(0.1);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.1);
	    trtSleepUntil(rel, dead);
	}
}

// --- define task 3  ----------------------------------------
void print1(void* args) {
    uint32_t rel, dead ;
	while(TRUE) {
		// wait for the message to be valid
		// Using trtAccept means that the task can keep running
		// if the message is not ready yet
		if(trtAccept(SEM_RX_WAIT)) {
			fprintf(stdout,"Input*2= %d \n\r", Message_vin*2) ;
			trtSignal(SEM_TX_WAIT); 
		}

		// Sleep
	    rel = trtCurrentTime() + SECONDS2TICKS(0.2);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.2);
	    trtSleepUntil(rel, dead);	
	}
}

/* void led1(void* args) 
  begin

	while(1)
	begin
		// wait on semaphore SEM_TASK1_WAIT 
		// which is signaled from task 2
		trtWait(SEM_TASK1_WAIT) ; 
		
		// blink the led
		PORTC = PORTC ^ 0x02 ;

		// update task count
		trtWait(SEM_SHARED) ;
		task_count++;
		trtSignal(SEM_SHARED);

	end
  end 

// --- define task 2  ----------------------------------------
void led2(void* args) 
  begin	
  	uint32_t rel, dead ;

	while(1)
	begin
		// If button zero is pushed, signal task1 to execute
		if (~PINB & (1<<PINB0)) 
			trtSignal(SEM_TASK1_WAIT) ;
		
		// blink the led
		PORTC = PORTC ^ 0x04 ;

		// update task count
		trtWait(SEM_SHARED) ;
		task_count++;
		trtSignal(SEM_SHARED);

		// Sleep
	    rel = trtCurrentTime() + SECONDS2TICKS(0.1);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.1);
	    trtSleepUntil(rel, dead);
	end
  end


// --- define task 3  ----------------------------------------
void print1(void* args) 
  begin	
    uint32_t rel, dead ;
	while(1)
	begin
		// blink the led
		PORTC = PORTC ^ 0x08 ;

		// update task count
		trtWait(SEM_SHARED) ;
		task_count++;
		trtSignal(SEM_SHARED);

		// wait for the message to be valid
		// Using trtAccept means that the task can keep running
		// if the message is not ready yet
		if(trtAccept(SEM_RX_WAIT)) 
		begin
			fprintf(stdout,"Input*2= %d \n\r", Message_vin*2) ;
			trtSignal(SEM_TX_WAIT); 
		end	

		// Sleep
	    rel = trtCurrentTime() + SECONDS2TICKS(0.2);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.2);
	    trtSleepUntil(rel, dead);	
	end
  end

// --- define task 4  ----------------------------------------
void print2(void* args) 
  begin
	uint16_t vin ;

	while(1)
	begin
		// blink the led
		PORTC = PORTC ^ 0x10 ;
		
		// message to task3 -- 
		// need two semaphores
		trtWait(SEM_TX_WAIT) ; // wait for the message to be received
		// exercise the uart
		fprintf(stdout, "enter integer>") ;
		fscanf(stdin, "%d", &vin) ;
		//trtWait(SEM_STRING_DONE); // waits for the <enter>
		fprintf(stdout,"%ld %ld %d\n\r", 
					trtCurrentTime(), task_count, vin) ;
		Message_vin = vin ;
		trtSignal(SEM_RX_WAIT) ; // tell receiver messsage is ready

		// update task count
		trtWait(SEM_SHARED) ;
		task_count++ ;
		trtSignal(SEM_SHARED);

		// kill the process when B.1 is pressed
		if (~PINB & (1<<PINB1)) 
			trtTerminate() ;
	end
  end
*/
// --- Main Program ----------------------------------
int main(void) {

  DDRC = 0xff;    // led connections
  PORTC = 0xff;
  DDRB = 0x00; // switch connections
  PORTB = 0xff; // pullup on

  //init the UART -- trt_uart_init() is in trtUart.c
  trt_uart_init();
  stdout = stdin = stderr = &uart_str;
  fprintf(stdout,"\n\r TRT 09feb09\n\r\n\r");

  // start TRT
  trtInitKernel(80); // 80 bytes for the idle task stack

  // --- create semaphores ----------
  // You must creat the first two semaphores if you use the uart
  trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; // uart receive ISR semaphore
  trtCreateSemaphore(SEM_STRING_DONE,0) ;  // user typed <enter>
  
  // Task synch
  trtCreateSemaphore(SEM_TASK1_WAIT, 0) ; // task2 controls task1 rate
  
  // message protection
  trtCreateSemaphore(SEM_TX_WAIT, 1) ; // message send interlock
  trtCreateSemaphore(SEM_RX_WAIT, 0) ; // message receive interlock
  
  // variable protection
  trtCreateSemaphore(SEM_SHARED, 1) ; // protect shared variable

 // --- creat tasks  ----------------
  /*
  trtCreateTask(led1, 100, SECONDS2TICKS(0.05), SECONDS2TICKS(0.05), &(args[0]));
  trtCreateTask(led2, 100, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
  trtCreateTask(print1, 100, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[2]));
  trtCreateTask(print2, 100, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[2]));
  */
  // --- Idle task --------------------------------------
  // For debugging, blink an LED
  // For production, you would probably comment out the body of the WHILE
  while (1) 
  begin
  	PORTC = PORTC ^ 0x80 ;
	_delay_ms(500) ;
  end

} // main
