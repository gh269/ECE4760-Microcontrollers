/*
Tasks to complete:

The LCD should be updated every 200 mSec or so.
An LED should blink about 1/second.
The capacitance should be measured as quickly as possible as described above.
The range of capacitances to be measured is 1 nf to 100 nf.
The program should detect whether a capacitance is present or not and display an appropriate message if no capacitor is present.
If present, format the capacitance as an ASCII number and prints the message C = xxx nf to the LCD
*
*
*
*/
#define F_CPU 16000000UL                
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "uart.h"

#define LED_BLINK_PERIOD 1000

//LED Bits

#define ONBOARD_LED 0x04; //LED is on D.2

//Timer TCCR0B Bits
#define T0B_CS02 4
#define T0B_CS01 2
#define T0B_CS00 1

volatile unsigned int led_time_count;


//1 ms timebase register
//Blinks LED 1/second
ISR (TIMER0_COMPA_vect){
	if( led_time_count == (LED_BLINK_PERIOD / 2) ){
		toggle_led();
	}

}


//Blinks the ONBOARD_LED D.2
void toggle_led(void){
	PORTD ^= ONBOARD_LED;
}

//setup timer 0 for a 1 ms timebase
// triggers the ISR on TIMER0_COMPA_vect
// on TCNT0 = OCR0A
void init_timer0(void){
	// Output capture/compare on OCR0A IE
	TIMSK0 = (1 << OCIE0A);
	OCR0A = 249;
	//T0BCS01 + T0BCS00 sets clk divider 64
	// 16 MHz 				250 KHz
	// ------  = 256 KHz;  ---------  = 1 KHz  = 1ms period
	//   64					OCR0A=249
	TCCR0B = T0B_CS01 + T0B_CS00;
	//turn on clear-on-match - timer A ISR will clear TCNT0 on match
	TCCR0A = (1 << WGM01);
}


void initialize(void){
	led_time_count = 0;
	init_timer0();

	DDRB = 0;
	DDRD = 0;

	//Enable LED Port
	DDRD |= ONBOARD_LED; //turn the LED to an output
	PORTD = 0; //turn off the LED 



	sei();
}

