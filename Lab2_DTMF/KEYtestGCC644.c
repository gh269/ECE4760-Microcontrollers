
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

//State machine state names


// The raw keyscan
unsigned char key ;   
// The decoded button number
unsigned char butnum ;

//key pad scan table
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
				//fprintf(stdout, "PINB is pressed.\n\r");
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
						play(0, 697);
						break;
					case 2: 
						play(0, 770);
						break;
					case 3: 
						play(0, 852);
						break;
					case 4: 
						play(0, 941);
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

	if (PushState == NoPush && !is_timed_playing) {
		//fprintf(stdout, "Depressed...\n\r");
		stop_playing();
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
			int i = 0;
			is_playing = FALSE;
			char silence;
			silence = FALSE;
			//init the playing state machine
			is_timed_playing = FALSE;
			while( i < 12){
				update_status_variables();
				if (mem[i] == 0) {
					break;
				}
				if(silence == FALSE && !is_playing){
					fprintf(stdout, "Int: %u\n\r", mem[i]);
					timed_play(high_freq[mem[i]], low_freq[mem[i]], 100);
					is_timed_playing = TRUE;
					fprintf(stdout, "Playing sound\n\r");
					silence = TRUE;
				}
				if(silence == TRUE && !is_playing) {
					timed_play(0, 0, 30);
					is_timed_playing = TRUE;
					fprintf(stdout, "Playing silence\n\r");
				}
				if(is_playing && dds_duration <= 0 && !silence){
					fprintf(stdout, "Playing timeout\n\r");
					stop_playing();
					is_timed_playing = FALSE;
					silence = TRUE;
					is_playing = FALSE;
				}
				if(is_playing && dds_duration <= 0 && silence){
					fprintf(stdout, "Silence timeout\n\r");
					stop_playing();
					is_timed_playing = FALSE;
					i++;
					silence = FALSE;
					is_playing = FALSE;
				}
			}
		}
		// A normal button press. 
		else {
			fprintf(stdout, "%u\n\r", butnum);
			if (mem_index < 12) {
				mem[mem_index] = butnum;
				mem_index++;
				play(high_freq[butnum], low_freq[butnum]);
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

  //play(1336, 852);
	
  //endless loop to read keyboard
  while(1) {
	update_status_variables();
	//Used for debouncing
    if (time1>=t1) {
		task1();
	}	
  }
}
