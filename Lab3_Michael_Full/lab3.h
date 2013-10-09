#include "trtSettings.h"
#include "trtkernel_1284.c"
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <string.h>

// COMMUNICATION AND SEMAPHOMRES

// serial communication library
// Don't mess with the semaphores
#define SEM_RX_ISR_SIGNAL 1
#define SEM_STRING_DONE 2 // user hit <enter>

// two semaphores to protect message --
// sender must wait until message is received 
// -- init to 1 becuase don't need to wait for first message
// receiver must wait until message is sent
#define SEM_RECV_IR  3
#define SEM_SEND_IR  4

// semaphore to protect shared variable
#define SEM_SHARED 5

// two semaphores to protect message --
// sender must wait until message is received 
// -- init to 1 becuase don't need to wait for first message
// receiver must wait until message is sent
#define SEM_RECV_IR  3
#define SEM_SEND_IR  4

// semaphore to protect shared variable
#define SEM_SHARED 5

// the usual
#define begin {
#define end }

// States
#define IDLE 0
#define CHALLENGE 1
#define REQUEST 2
#define RESPONSE 3
uint8_t state;

// Configuration switch states
uint8_t key_on;
uint8_t lock_on;

// Global key variables
char key_id;
char request;
uint8_t button_pushed;

// a value passed between task3 and task4
uint16_t Message_vin ;

// shared variable to count number of total task invocations
uint32_t task_count, error_count ;


// IR COMMUNICATION

/* CPU frequency */
#define F_CPU 16000000UL
/* UART baud rate */
// for IR link
#define IR_UART_BAUD  4800
#define PC_UART_BAUD  9600

#define begin {
#define end }

// ISR
#define SUSPEND cli();
#define RESUME sei();
 
//the task subroutine
void send_task(void);  
void recv_task(void);  	// specify transmitter
void recv_any_task(void);  	// any transmitter
void initialize_ir(void); //all the usual mcu stuff 

//timeout counter  
// time to give up and assume you will never get a response   
#define ir_rx_timeout 100 //milliseconds
#define ir_tx_timeout 100
   
// task timers and global time counters (mSec)
volatile unsigned long tx_send_time, rx_rec_time ;

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

// Prototypes
void IR_key( void* args );
void IR_lock( void* args );
