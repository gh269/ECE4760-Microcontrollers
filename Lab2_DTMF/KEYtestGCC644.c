                 
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
#include "dds.h"
#include "atmega1284p.h"

#define maxkeys 12
#define PORTDIR DDRA
#define PORTDATA PORTA
#define PORTIN PINA

//timeout values for each task
#define t1 20
//volatile unsigned int time1;	//timeout counters 
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

unsigned int mem[12] = {0, 0, 0, 0, 0, 0,
			            0, 0, 0, 0, 0, 0};
unsigned int mem_index;

unsigned int high_freq[12] = {1209, 1336, 1477, 
							  1209, 1336, 1477, 
							  1209, 1336, 1477, 
							  1209, 1336, 1477};
unsigned int low_freq[12] = {697, 697, 697, 
							 770, 770, 770, 
							 852, 852, 852, 
							 941, 941, 941};

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
			//Test mode override
			if (~PINB & 0x01) {
		 		PushFlag=0;			
		   	}  
        }
        else PushState=NoPush;
        break;
     case Pushed:  
        if (butnum != 0) {
			PushState=Pushed;
			//Test mode override
			if (~PINB & 0x01) {
		 		switch (butnum) {
					case 1: 
						play(697, 0, 100);
						break;
					case 2: 
						play(770, 0, 100);
						break;
					case 3: 
						play(852, 0, 100);
						break;
					case 4: 
						play(941, 0, 100);
						break;
					case 5:
						play(1209, 0, 100);
						break;
					case 6:
						play(1336, 0, 100);
						break;
					case 7: 
						play(1477, 0, 100);
						break;
					default:
						break;
				}	
		   	}  
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
		// The * button was pressed. Clear all memory.
		if (butnum == 10) {
			for (int i = 0; i < 12; i++) {
				mem[i] = 0;
			}
			mem_index = 0;
		}
		// The # button was pressed. Play all sounds in memory.
		else if (butnum == 12) {
			for (int i = 0; i < 12; i++) {
				if (mem[i] != 0) {
					fprintf(stdout, "%u\n\r", mem[i]);
					//play(high_freq[mem[i]], low_freq[mem[i]], 100);
					//play(0, 0, 30);
				}
			}
		}
		// A normal button press. 
		else {
			if (mem_index < 12) {
				mem[mem_index] = butnum;
				mem_index++;
				//play(high_freq[butnum], low_freq[butnum], 1000);
			}		
		}
		// For debugging purposes without sound.
		for (int i = 0; i < 12; i++) {
			fprintf(stdout, "%d ", mem[i]);
		}
		fprintf(stdout, "\n\r");
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
	// PORT B is an input
	DDRB = 0x00;
	//for no button push
	PushFlag = 0;
	//init the state machine
	PushState = NoPush;

	mem_index = 0;

	init_dtmf();
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
  	}
  }
