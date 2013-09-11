#define F_CPU 16000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <util/delay.h> // needed for lcd_lib
#include "lcd_lib.h"


/*
	Your program will have to (in time order):

	Set PortB3 to an input.
	Drive PortB2 to zero by making it an output and wait long enough to discharge the capacitor through 100 ohms. Clearly, to dischage to zero volts with 1% accuracy, R2>100*(100ohms).
	Convert PortB2 to an input and start a timer. The capacitor will start to charge toward Vcc.
	Detect when the voltage at PortB2 is greater than than the voltage at PortB3. That is, you will have to record
	when the comparator changes state. You could do this by polling the ACO bit of the ACSR and stopping the clock when ACO changes state, but a much better way to do it is to use the timer1 input capture function set up to be triggered by the comparator. Using input capture gives better timing accuracy and more dynamic range.
	Repeat
*/

#define TRUE 1
#define FALSE 0


//-----------CAPACITANCE MEASUREMENT BITS

// 0000 1000 - B3
#define COMPARATOR_REFERENCE 0x08
//B2 0000 0100
#define COMPARATOR_INPUT 0x04

//Discharge Period [ units: us ]
#define CAP_DISCHARGE_PERIOD 90

// Each of each count (16Mhz) [ units: ns]
#define T1_CLK_PERIOD 62.5

// The R2 resistor value [ units: Ohms]
//brown-black-orange-gold
#define RESISTOR 10000

//--------------LED 1 HZ Blinking--------
//1 Hz LED Blinking
#define ONBOARD_LED 0x04
// units: ms
#define LED_BLINK_PERIOD 1000

//---------------Timer TCCR0B Bits-------------
#define T0B_CS02 4
#define T0B_CS01 2
#define T0B_CS00 1
#define CLEAR_ON_MATCH (1 << WGM01)
#define OUTPUT_COMPARE_A0_MATCH_INTERRUPT_ENABLE (1 << OCIE0A)
#define OUTPUT_COMPARE_A1_MATCH_INTERRUPT_ENABLE (1 << OCIE1A)

//---------------LCD variables------------------
//units : ms
#define LCD_REFRESH_RATE 200

const uint8_t LCD_initialize[] PROGMEM = "LCD Initialized\0";
const uint8_t LCD_number[] PROGMEM = "Capacitance=\0";
uint8_t lcd_buffer[17];	// LCD display buffer
uint16_t count;			// a number to display on the LCD  
uint8_t anipos, dir;	// move a character around  


// Flags for the finite state machine transitions
volatile unsigned int begin_cap_measurement = FALSE;
volatile unsigned int cap_discharged = FALSE;
volatile unsigned int cap_charged = FALSE;

//time counter for LED blinking
volatile unsigned int led_time_count;
//time counter for LCD refresh
volatile unsigned int lcd_time_count;

//time counter to Validate Cap Discharge
//this counter performs the 100 - 10,000 cycle wait to ensure that the capacitor 
//starts discharged.
//volatile unsigned int validate_cap_discharge_time_count;

// timer 1 capture variable for computing charging time	
volatile double charge_time; 
// variable to store capacitance for print out
volatile double capacitance;
// precomputed log(.5) needed for capacitance calculation
const double ln_half = 0.6931471805599453;

//Timer0A is 1ms time base for on board LED and LCD
//screeen refresh
void init_timer0A(void){
	TIMSK0 = OUTPUT_COMPARE_A0_MATCH_INTERRUPT_ENABLE;
	OCR0A = 249;
	TCCR0B = 3;
	TCCR0A = CLEAR_ON_MATCH;
}

//Blinks the ONBOARD_LED D.2
void toggle_led(void){
	PORTD ^= ONBOARD_LED;
}

//1 ms timebase register
//Blinks LED 1/second
//refreshes LCD 1/200 ms
ISR (TIMER0_COMPA_vect){
	if( led_time_count > 0)
		--led_time_count;
	if( lcd_time_count > 0)
		--lcd_time_count;

}

void initialize(void){
	anipos = 8;
	led_time_count = 0;
	init_timer0A();
	//DDRB = 0;
	DDRB = 0;
	DDRD = 0;

	DDR = ONBOARD_LED;
	PORTD = 0xFF;

	cap_discharged = FALSE;
	begin_cap_measurement = FALSE;
	cap_charged = FALSE;

	init_lcd();
	LCDclr();
	sei();
}
// LCD setup
void init_lcd(void){
	LCDinit();	//initialize the display
	LCDcursorOFF();
	LCDclr();				//clear the display
	LCDGotoXY(0,0);
	CopyStringtoLCD(LCD_number, 0, 0);
}


// writes contents of lcd_buffer to LCD every 200 mSec
// 
void refresh_lcd(void){
  // increment time counter and format string 
  //if (capacitance >= .1 && capacitance <= 100) {
  sprintf(lcd_buffer,"%-.5f",capacitance);	 
  //}
  //else {
  //	sprintf(lcd_buffer,"N/A");
  //}               
  LCDGotoXY(0, 1);
  	// display the capacitance 
  LCDstring(lcd_buffer, strlen(lcd_buffer));	
  /*
  if (capacitance >= .09 && capacitance <= 101) {
  	sprintf(lcd_buffer, "%-f", capacitance);	 
  }
  else {
  	sprintf(lcd_buffer, "N/A");
  } 
  */         
  // now move a char left and right
  LCDGotoXY(anipos,1);	   //second line
  LCDsendChar(' '); 
      	
  if (anipos>=15) dir=-1;   // check boundaries
  if (anipos<=8 ) dir=1;
  anipos=anipos+dir;
  LCDGotoXY(anipos,1);	   //second line
  LCDsendChar('o');
}
int main(void){
	initialize();
	CopyStringtoLCD(LCD_number, 0, 0);//start at char=0 line=0
	
	while(1){
		if( led_time_count == 0){
			led_time_count = LED_BLINK_PERIOD / 2;
			toggle_led();
		}
		if( lcd_time_count == 0){
			lcd_time_count = LCD_REFRESH_RATE;
			refresh_lcd();
		}
	}
}


