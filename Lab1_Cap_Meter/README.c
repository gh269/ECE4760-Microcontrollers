// Mega 644 version

//Run all three timers in different modes
// timer 0 -- compare-match timebase
// timer 1 -- input capture from analog comparator
// timer 2 -- stand alone square wave gen  

// main will schedule task1, and poll the ACSR ACO bit
// task 1 will  print the periods 
// timer0 compare ISR will increment a timer
// timer1 capture ISR will compute a period from the capture

// MUST connect port B.3 (AIN1) to port D.7 (OC2)
// Capture yields cycle-accurate period measurement
// Polling the ACO bit yields 20 cycle variability 
#define F_CPU 16000000UL                
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

//set up the debugging utility ASSERT
//#define __ASSERT_USE_STDERR
//#include <assert.h>

#include "uart.h"

// UART file descriptor
// putchar and getchar are in uart.c
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
       

//time for task
#define t1 250  

#define begin {
#define end }
 
//the task subroutine
void task1(void);  	//execute at 2 Hz

void initialize(void); //all the usual mcu stuff 

//timeout counter for task1         
volatile unsigned char time1; 

// timer 1 capture variables for computing sq wave period	
volatile unsigned int T1capture, lastT1capture, period ; 

// polling variables for computing sq wave period (low accuracy)
unsigned int periodPoll, T1poll, lastT1poll ;

//comparator output bit
char ACObit, lastACObit ;
         
//**********************************************************
//timer 0 overflow ISR
ISR (TIMER0_COMPA_vect) 
begin      
  //Decrement the time if not already zero
  if (time1>0)	--time1;
end  

//**********************************************************
// timer 1 capture ISR
ISR (TIMER1_CAPT_vect)
begin
    // read timer1 input capture register
    T1capture = ICR1 ; 
    // compute time between captures
    period =  T1capture - lastT1capture;
    lastT1capture = T1capture ;
end

//**********************************************************       
//Entry point and task scheduler loop
int main(void)
begin  
  initialize();
  
  // main task scheduler loop 
  while(1)
  begin
  	// task1 prints the period     
    if (time1==0){time1=t1;	task1();}

    // poll for ACO 0->1 transition  (ACSR.5)
	// as fast as possible and record Timer1 
    ACObit = ACSR & (1<<ACO) ;
    if ((ACObit!=0) && (lastACObit==0))
    begin
        T1poll = TCNT1 ;
        periodPoll = T1poll - lastT1poll ;
        lastT1poll = T1poll ; 
    end
    lastACObit = ACObit ;
  end
end  
  
//**********************************************************          
//Timed task subroutine
//Task 1
void task1(void) 
begin  
    //reset the task timer
  
  // print capture interval 
  fprintf(stdout,"%d %d\n\r", period, periodPoll);

end  
 
//********************************************************** 
//Set it all up
void initialize(void)
begin
           
  //set up timer 0 for 1 mSec timebase 
  TIMSK0= (1<<OCIE0A);	//turn on timer 0 cmp match ISR 
  OCR0A = 249;  		//set the compare re to 250 time ticks
  //set prescalar to divide by 64 
  TCCR0B= 3; //0b00001011;	
  // turn on clear-on-match
  TCCR0A= (1<<WGM01) ;
  
  //set up timer1 for full speed and
  //capture an edge on analog comparator pin B.3 
  // Set capture to positive edge, full counting rate
  TCCR1B = (1<<ICES1) + 1; 
  // Turn on timer1 interrupt-on-capture
  TIMSK1 = (1<<ICIE1) ;

  // Set analog comp to connect to timer capture input 
  // and turn on the band gap reference on the positive input  
  ACSR = (1<<ACBG) | (1<<ACIC) ; //0b01000100  ;
  // Comparator negative input is B.3
  DDRB = 0 ;
  
  // set up timer 2 for square wave with NO ISR
  // 200 cycle period from OC2
  OCR2A = 99 ; //100 cycles/half-period
  //  count at full rate
  TCCR2B =  1;	
  // set to toggle OC2A, clear on match,
  TCCR2A = (1<<COM2A0) | (1<<WGM21) ;
  // PORT D.7 is OC2A
  DDRD = (1<<PIND7) ;	
  
  
  //init the task timer
  time1=t1;
      
  //init the UART -- uart_init() is in uart.c
  uart_init();
  stdout = stdin = stderr = &uart_str;
  fprintf(stdout,"Starting timers...\n\r");
      
  //crank up the ISRs
  sei();

end  
//==================================================

   
