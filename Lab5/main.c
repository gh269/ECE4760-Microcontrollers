#include "trtSettings.h"
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>

#include "trtkernel_1284.c"
#include "trtUart.h"
#include "trtUart.c"
#include "lcd_lib.h"
#include "lcd_lib.c"


/********************************************************************/
//						UART Functions
/********************************************************************/
FILE uart0 = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

/********************************************************************/
//						FSM Variables
/********************************************************************/
// semaphore to protect shared variable
// semaphores 1-3 are used by the UART
#define SEM_SHARED 4
// input arguments to each thread
// not actually used in this example
int args[2] ;

// binary states
#define FALSE 0
#define TRUE 1

// relay & LED bits
#define LED_EN 	 0x04
#define RELAY_EN 0x08

// input and output variables
volatile int cTemp;		// current temperature
volatile int dTemp;		// desired temperature
volatile int time_rem;  // time remaining in seconds

// timer variable
volatile int msec;
volatile int count_en;

// lcd variables
const int8_t LCD_line1[] PROGMEM = "Current:\0";
const int8_t LCD_line2[] PROGMEM = "Desired:\0";
int8_t lcd_buffer[17];	// LCD display buffer

/********************************************************************/
// 							ISRs & Helper Functions
/********************************************************************/
// --- Timer ISR ------------------------
ISR (TIMER0_COMPA_vect) {
	trtWait(SEM_SHARED);
	if ((time_rem > 0) && count_en) {
		if (msec < 1000) {
			msec++;
		}
		else {
			msec = 0;
			time_rem--;
			if (time_rem == 0) {
				count_en = 0;
			}
		}
	}
    trtSignal(SEM_SHARED);
}


//**********************************************************
// LCD setup
void init_lcd(void) {
	LCDinit();				//initialize the display
	LCDcursorOFF();
	LCDclr();				//clear the display
	LCDGotoXY(0,0);
}

//********************************************************** 
// ADC setup
void adc_init(void){
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));    //16Mhz/128 = 125Khz the ADC reference clock
	ADMUX |= (1<<REFS0);                //Voltage reference from Avcc (5v)
	ADCSRA |= (1<<ADEN);                //Turn on ADC
	ADCSRA |= (1<<ADSC);                //Do an initial conversion because this one is the slowest and to ensure that everything is up and running
}
 
uint16_t read_adc(uint8_t channel){
	ADMUX &= 0xF0;                  //Clear the older channel that was read
	ADMUX |= channel;               //Defines the new ADC channel to be read
	ADCSRA |= (1<<ADSC);            //Starts a new conversion
	while(ADCSRA & (1<<ADSC));     	//Wait until the conversion is done
	return ADCW;                    //Returns the ADC value of the chosen channel
}
// http://hekilledmywire.wordpress.com/2011/03/16/using-the-adc-tutorial-part-5/

//********************************************************** 
//Set it all up
void initialize(void) {
	DDRA = 0x00;		// ADC Port
	PORTA = 0x00;
	DDRC = 0xff;    	// LCD connections
	PORTC = 0x00;
	DDRD |= 0x0C;		// LED status light && Relay
	PORTC |= 0x08;		// Initialize relay to high

	// ******************** 
	//initialize variables
	trtWait(SEM_SHARED);
	cTemp = 0;
	dTemp = 0;
	time_rem = 0;
	msec = 0;
	count_en = 0;
	trtSignal(SEM_SHARED);

	// ******************** 
	//init LCD
	init_lcd();
	LCDclr();
	// put some stuff on LCD
	CopyStringtoLCD(LCD_line1, 0, 0);//start at char=0 line=0
	CopyStringtoLCD(LCD_line2, 0, 1); 

	// ********************
	//init ADC
	adc_init();

	// ******************** 
	//set up timer 0 for 1 mSec timebase 
	TIMSK0= (1<<OCIE0A);	//turn on timer 0 cmp match ISR 
	OCR0A = 249;  		//set the compare reg to 250 time ticks
	//set prescalar to divide by 64 
	TCCR0B= 3; 	
	// turn on clear-on-match
	TCCR0A= (1<<WGM01) ;

	// ********************
	//crank up the ISRs
	sei();
}  

/********************************************************************/
// 							FSM Tasks
/********************************************************************/
// --- define task 1  ----------------------------------------
void serialComm(void* args) {
	// Declare the command and num variables
	volatile int num ;
	char cmd[4] ;
    // initialize
    initialize();
	while (TRUE) {
		// commands:
		// 'temp' sets the desired temperature
		// 'time' sets the desired time
		fprintf(stdout, ">") ;
		fscanf(stdin, "%s%u", cmd, &num) ;
		// update shared variables
		trtWait(SEM_SHARED) ;
		if (strcmp(cmd, "temp") == 0) {
			if (num < 0) {
				fprintf(stdout, "Please input a positive temperature value.");
			}
			else {
				dTemp = num;
			}
		}
		if (strcmp(cmd, "time") == 0) {
			if (num < 0) {
				fprintf(stdout, "Please input a positive time value in seconds.");
			}
			else {
				time_rem = num;
				msec= 0;
			}
		}
		// 6 Minute Egg Mode
		if (strcmp(cmd, "egg") == 0) {
			cTemp = 100;
			time_rem = 3600;
		}
		trtSignal(SEM_SHARED);
	}
}

// --- define task 2  ----------------------------------------
void lcdComm(void* args) {
	uint32_t rel, dead;
	// increment time counter and format string 
	while (TRUE) {
	  // display the current temp
	  trtWait(SEM_SHARED) ;
	  sprintf(lcd_buffer, "%iC      ", cTemp);
	  LCDGotoXY(9, 0);
	  LCDstring(lcd_buffer, strlen(lcd_buffer));
	  // display the desired temp 
	  sprintf(lcd_buffer, "%iC      ", dTemp);
	  //sprintf(lcd_buffer, "%is      ", time_rem);
	  LCDGotoXY(9, 1);
	  LCDstring(lcd_buffer, strlen(lcd_buffer));
	  trtSignal(SEM_SHARED);
	  // sleep
	  rel = trtCurrentTime() + SECONDS2TICKS(0.25);
	  dead = trtCurrentTime() + SECONDS2TICKS(0.5);
	  trtSleepUntil(rel, dead);	
  	}
}

// --- define task 3  ----------------------------------------
void adjustTemp(void* args) {
	uint32_t rel, dead;
	uint16_t adc_in;
	while(TRUE){
		// Read ADC value
		adc_in = read_adc(0);
		//fprintf(stdout, "ADC: %i\n\r", adc_in);

		// Control mechanism
		trtWait(SEM_SHARED);
		cTemp = (adc_in + 3) / 2.1;
		if (cTemp < 0) cTemp = 0; 
		if (cTemp < (dTemp /* * .95 */)) {	// Factor of .9 to account for carryover effect
			PORTD &= ~RELAY_EN;		// Turn on heating element
			PORTD &= ~LED_EN;		// Turn off LED
		}
		else {
			PORTD |= RELAY_EN;		// Turn off heating element
			PORTD |= LED_EN;    	// Turn on LED
			count_en = 1;			// Enable timer count down
		}
		trtSignal(SEM_SHARED);

		// sleep
	  	rel = trtCurrentTime() + SECONDS2TICKS(2);
	  	dead = trtCurrentTime() + SECONDS2TICKS(4);
	  	trtSleepUntil(rel, dead);	
	}
}

// --- Main Program ----------------------------------
int main(void) {
  //init the UART -- trt_uart_init() is in trtUart.c
  trt_uart_init();
  stdout = stdin = stderr = &uart0;
  fprintf(stdout,"\n\r Welcome to KitchenBot UI \n\r Please input your instructions below\n\r The options are: time, temp, & egg\n\r\n\r");
    // start TRT
  trtInitKernel(80); // 80 bytes for the idle task stack

  // --- create semaphores ----------
  // You must create the first two semaphores if you use the uart
  trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; // uart receive ISR semaphore
  trtCreateSemaphore(SEM_STRING_DONE,0) ;  // user typed <enter>

  // variable protection
  trtCreateSemaphore(SEM_SHARED, 1) ; // protect shared variable

  // --- create tasks  ----------------
  trtCreateTask(serialComm, 2000, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[0]));
  trtCreateTask(lcdComm, 2000, SECONDS2TICKS(0.25), SECONDS2TICKS(0.5), &(args[0]));
  trtCreateTask(adjustTemp, 2000, SECONDS2TICKS(2), SECONDS2TICKS(4), &(args[0]));

  // --- Main Idle task --------------------------------------
  while (1) {
  	// Do nothing
  }
} // main