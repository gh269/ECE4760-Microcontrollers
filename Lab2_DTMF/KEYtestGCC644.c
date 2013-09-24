                 
// Keyscanner
// Mega644 version 
// assumes a standard 4x4 keypad connected to a port
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
      
#include "uart.h"

#define maxkeys 12
#define PORTDIR DDRA
#define PORTDATA PORTA
#define PORTIN PINA

//timeout values for each task
#define t1 20
volatile unsigned int time1;	//timeout counters 
unsigned char PushFlag;		//message indicating a button push 
unsigned char PushState;	//state machine  
//State machine state names
#define NoPush 1 
#define MaybePush 2
#define Pushed 3
#define MaybeNoPush 4

// The raw keyscan
unsigned char key ;   
// The decoded button number
unsigned char butnum ;

//key pad scan table
/*unsigned char keytbl[16] = {0x77, 0x7b, 0x7d, 0x7e,
							0xb7, 0xbb, 0xbd, 0xbe,
							0xd7, 0xdb, 0xdd, 0xde,
							0xe7, 0xeb, 0xed, 0xee}; */

unsigned char keytbl[16] = {0x77, 0x7b, 0x7d, 0xb7,
							0xbb, 0xbd, 0xd7, 0xdb,
							0xdd, 0xe7, 0xeb, 0xed,
							0xff, 0xff, 0xff, 0xff};

// UART file descriptor
// putchar and getchar are in uart.c
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

//**********************************************************
//timer 0 comare match ISR
ISR (TIMER0_COMPA_vect) {
  //Decrement the  time if they are not already zero
  if (time1>0) 	--time1;
}

 
//******************************* 
//Task 1  
void task1(void) {
	time1=t1;  //reset the task timer

	//get lower nibble
	PORTDIR = 0x0f;
	PORTDATA = 0xf0; 
	_delay_us(5);
	key = PORTIN;
  
	//get upper nibble
	PORTDIR = 0xf0;
	PORTDATA = 0x0f; 
	_delay_us(5);
	key = key | PORTIN;
  
	//find matching keycode in keytbl
	if (key != 0xff) {
	  for (butnum=0; butnum<maxkeys; butnum++) {   
	  	if (keytbl[butnum]==key)  break;   
	  }

	  if (butnum==maxkeys) butnum=0;
	  else butnum++;	   //adjust by one to make range 1-16
	}
	else butnum=0;
	
	// Switching in the finite state machine.
	switch (PushState) {
     case NoPush: 
        if (butnum != 0) PushState=MaybePush;
        else PushState=NoPush;
        break;
     case MaybePush:
        if (butnum != 0) {
           PushState=Pushed;   
           PushFlag=1;
        }
        else PushState=NoPush;
        break;
     case Pushed:  
        if (butnum != 0) {
			PushState=Pushed;
		}
        else PushState=MaybeNoPush;    
        break;
     case MaybeNoPush:
        if (butnum != 0) PushState=Pushed; 
        else {
           PushState=NoPush;
           PushFlag=0;
        }    
        break;
  	}

	if (PushFlag) {
		PushFlag = 0;
		fprintf(stdout, "%d\n\r",butnum);
	}
} 
 
//Initialization used for the timer interrupts for debouncing
void initialize(void) {
	//set up timer 0 for 1 mSec timebase 
	OCR0A = 249;  		//set the compare re to 250 time ticks
	TIMSK0= (1<<OCIE0A);	//turn on timer 0 cmp match ISR 
	//set prescalar to divide by 64 
	TCCR0B= 3; //0b00000011;	
	// turn on clear-on-match
	TCCR0A= (1<<WGM01) ;

	//init the task timers
	time1=t1;  

	//for no button push
	PushFlag = 0;
	//init the state machine
	PushState = NoPush;

	//crank up the ISRs
	sei() ;
}

int main(void) {
  // Init port B to show keyboard result
  DDRB = 0xff;
  // and turn out the LEDs
  PORTB = 0xff;   
  
  // init the UART
  uart_init();
  stdout = stdin = stderr = &uart_str;
  fprintf(stdout, "Starting...\n\r");

  initialize();

  //endless loop to read keyboard
  while(1) {
	//Used for debouncing
    if (time1==0) {
	  	//fprintf(stdout, "Entering task2...\n\r");	
		task1();
	}

	/*  	
	// init the DDS phase increment
    // for a 32-bit DDS accumulator, running at 16e6/256 Hz:
    // increment = 2^32*256*Fout/16e6 = 68719 * Fout
    // Fout=1000 Hz, increment= 68719000 
    // int increment = 68719000L ;
	switch (butnum) {
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		case 4: 
			break;
		case 5:
			break;
		case 6:
			break;
		case 7:
			break;
		case 8:
			break;
		case 9: 
			break;
		case 10: 
			break;
		case 11:
			break;
		case 12:
			break;
		case 13:
			break;
		case 14:
			break;
		case 15:
			break;
		case 16:
			break;
		default:
			break;
	}
 	*/
  	}
  }
