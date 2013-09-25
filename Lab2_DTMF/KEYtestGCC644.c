
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

//******************************* 
//Task 1  
void task1(void) {
	time1=0;  //reset the task timer

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
        if (butnum != 0) {
			PushState=MaybePush;
		}
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
						play(697, 0);
						break;
					case 2: 
						play(770, 0);
						break;
					case 3: 
						play(852, 0);
						break;
					case 4: 
						play(941, 0);
						break;
					case 5:
						play(1209, 0);
						break;
					case 6:
						play(1336, 0);
						break;
					case 7: 
						play(1477, 0);
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

	if (PushState == NoPush) {
		fprintf(stdout, "Depressed...\n\r");
		//stop_playing();
	}	

	if (PushFlag) {
		PushFlag = 0;
		// The * button was pressed. Clear all memory.
		if (butnum == 10) {
			fprintf(stdout, "%u\n\r", butnum);
			for (int i = 0; i < 12; i++) {
				mem[i] = 0;
			}
			mem_index = 0;
		}
		// The # button was pressed. Play all sounds in memory.
		else if (butnum == 12) {
			fprintf(stdout, "%u\n\r", butnum);
			for (int i = 0; i < 12; i++) {
				if (mem[i] != 0) {
					//play(high_freq[mem[i]], low_freq[mem[i]]);
				}
			}
		}
		// A normal button press. 
		else {
			if (mem_index < 12) {
				fprintf(stdout, "%u\n\r", butnum);
				mem[mem_index] = butnum;
				mem_index++;
				//play(high_freq[butnum], low_freq[butnum]);
			}		
		}
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
	time1=0;  
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
  
  play(1477, 697);
  //endless loop to read keyboard
  while(1) {
	// Used for keeping track of time.
	if( count <= 0){
		count = COUNTMS;
		//time++;
		time1++;
	}

	//Used for debouncing
    if (time1>=t1) {
		task1();
	}
	/*
	if(time >= dds_duration){
		time = 0;
		OCR0A = 0;
		stop_playing();
	}
	*/
	if(changed == TRUE){

		changed = FALSE;

		sample++;
		//ramping up
		if( sample <= RAMPUPEND )
			rampCount++;
		//holdsteady the max value 
		else if( sample > RAMPUPEND && sample <= RAMPDOWNSTART)
			rampCount = 255;
		//begin rampdown
		else if( sample > RAMPDOWNSTART && sample <= RAMPDOWNEND)
			rampCount--;
		//finished ramping
		else if(sample > RAMPDOWNEND){
			rampCount = 0;
			sample = 0;

		}
		else{
			continue;
		}
	}

  }
}
