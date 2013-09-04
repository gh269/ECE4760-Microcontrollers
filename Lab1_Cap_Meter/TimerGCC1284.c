// Mega1284 version
// July 2012
//
// Connections:
// buttons on d.6 and d.7 with 330 ohm series resistors to ground
// leds on b.0 and b.1 with 330 ohm series resistors to ground
// USB cable from type B conenctor to PC running PuTTY.
//
// Function:
// Task 1 blinks an LED on b.0 at either 2 or 8 Hz
// Task 2 blinks a LED on b.1 at 1 Hz 
//   and blinks the onboard LED
//   and prints the current time and time in eeprom
// Task 3 records button presses
//
#define F_CPU 16000000UL
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/eeprom.h> 

// serial communication library
#include "uart.h"

//timeout values for each task
#define t1 250  
#define t2 500
#define t3 60  
#define begin {
#define end }
 
// eeprom defines addressee
#define eeprom_true 0
#define eeprom_data 1

//the three task subroutines
void task1(void);  	//blink at 2 or 8 Hz
void task2(void);	//blink at 1 Hz
void task3(void);	//detect buttons and modify task 1  

void initialize(void); //all the usual mcu stuff 
          
volatile unsigned int time1, time2, time3;	//timeout counters 
unsigned char tsk3m1, tsk3m2;				//task 3 message to task 1
unsigned char led;					//light states 
unsigned int time ;				//time since boot 


// UART file descriptor
// putchar and getchar are in uart.c
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
         
//**********************************************************
//timer 0 compare ISR
ISR (TIMER0_COMPA_vect)
begin      
  //Decrement the three times if they are not already zero
  if (time1>0)	--time1;
  if (time2>0) 	--time2;
  if (time3>0)	--time3;
end  

//**********************************************************       
//Entry point and task scheduler loop
int main(void)
begin  
  initialize();

  // find out if eeprom has EVER been written
  if (eeprom_read_byte((uint8_t*)eeprom_true) != 'T')
  begin // if not, write some data and the "written" flag
  	eeprom_write_word((uint16_t*)eeprom_data,0);
	eeprom_write_byte((uint8_t*)eeprom_true,'T');
  end
  
   //main task scheduler loop 
  while(1)
  begin 
  	// reset time and call task    
    if (time1==0){ time1=t1; task1();}
    if (time2==0){ time2=t2; task2();}
    if (time3==0){ time3=t3; task3();}
  end

end  
  
//*******************************         
//Task subroutines
//Task 1
void task1(void) 
begin  
  //check for task 3 message and speed up 
  if (tsk3m1 != 0) time1 >>= 2;  

  //toggle the zeros bit if button up
  if (!tsk3m2) led = led ^ 0x01;

  PORTB = led;
end  
 
//******************************* 
//Task 2  
void task2(void) 
begin
     //toggle the ones bit
  	led = led ^ 0x02;
  	PORTB = led;

	// blink the onboard LED
	PORTD ^= 0x04 ;

	// and print the current time
	// and the saved time
	fprintf(stdout,"%d %d \n\r", time++,
				eeprom_read_word((uint16_t*)eeprom_data)) ;
end  
 
//*******************************   
//Task 3  
void task3(void) 
begin
  // detect D.7 button push and
  // generate the message for task 1
  tsk3m1 = ~PIND & 0x80;  
  // record the time to check eeprom writing
  if (tsk3m1) eeprom_write_word((uint16_t*)eeprom_data,time);
  // next message
  tsk3m2 = ~PIND & 0x40;
end 
  
//********************************************************** 
//Set it all up
void initialize(void)
begin
  //set up the ports
  DDRB=0x03;    // PORT b.0 and b.1 are outputs  
  PORTB=0;
  DDRD=0x02;	// PORT D is an input except for UART on D.1
  DDRD |= 0x04;  // and d.2 which runs another LED
  PORTD = 0xc0;  // turn on pullups on d.6 and d.7 for pushbuttons
       
  //set up timer 0 for 1 mSec timebase 
  TIMSK0= (1<<OCIE0A);	//turn on timer 0 cmp match ISR 
  OCR0A = 249;  		//set the compare reg to 250 time ticks
  //set prescalar to divide by 64 
  TCCR0B= 3; 	
  // turn on clear-on-match
  TCCR0A= (1<<WGM01) ;

  //init the LED status (all off)
  led=0x00; 
  
  //init the task timers
  time1=t1;
  time2=t2;
  time3=t3;    
  
  //init the task 3 message for no message
  tsk3m1=0;
  tsk3m2=0;

  //init the UART -- uart_init() is in uart.c
  uart_init();
  stdout = stdin = stderr = &uart_str;
  fprintf(stdout,"Starting...\n\r");
  
  //crank up the ISRs
  sei();
end  

   
