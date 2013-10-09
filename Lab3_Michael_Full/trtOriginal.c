// test TRT with two led blinking tasks
// and two UART tasks

#include "trtSettings.h"
#include "trtkernel_1284.c"
#include <util/delay.h>
#include <stdio.h>

// serial communication library
// Don't mess with the semaphores
#define SEM_RX_ISR_SIGNAL 1
#define SEM_STRING_DONE 2 // user hit <enter>
#include "trtUart.h"
#include "trtUart.c"
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

// --- define task 1  ----------------------------------------
  void led1(void* args) 
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
  trtCreateTask(led1, 100, SECONDS2TICKS(0.05), SECONDS2TICKS(0.05), &(args[0]));
  trtCreateTask(led2, 100, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
  trtCreateTask(print1, 100, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[2]));
  trtCreateTask(print2, 100, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[2]));

  // --- Idle task --------------------------------------
  // For debugging, blink an LED
  // For production, you would probably comment out the body of the WHILE
  while (1) 
  begin
  	PORTC = PORTC ^ 0x80 ;
	_delay_ms(500) ;
  end

} // main
