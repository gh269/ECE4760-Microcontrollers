#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>


#include "atmega1284p.h"
#include "dds.h"

#define OUTPUT_PIN (1 << PINB3);

volatile unsigned long accumulator_a;
volatile unsigned long accumulator_b;

unsigned char highbyte;

unsigned long increment_a;
unsigned long increment_b;

volatile unsigned long sineA;
volatile unsigned long sineB;

volatile unsigned char highbyte_a;
volatile unsigned char highbyte_b;

volatile unsigned int  sample;

#define RAMP_LENGTH 312

signed char sineTable[256];
/*
Timer0 is a 8 bit register 
and will overflow at a rate of 

16 MHz 
------ = 62,500 Hz
 256 
 To generate a 1ms time base, 
*/

volatile unsigned long accumulator;
volatile unsigned char changed;

volatile unsigned long increment;
ISR( TIMER0_OVF_vect){
	changed = TRUE;

	OCR0A = 128 + 
	   ( (
	   	   (sineTable[(accumulator_a >> 24)] + sineTable[(accumulator_b >> 24)]) * rampTable[rampCount]
	   	 ) >> 8
	   );

	accumulator_a += increment_a;
	accumulator_b += increment_b;
	//generates a 1 ms timebase
	// Used for keeping track of time.
	if( count <= 0){
		count = COUNTMS;
		time1++;
		if( is_timed_playing && dds_duration > 0){
			dds_duration--;
		}
	}
	count--;
	if( play_start){
		rampCount++;
	}

	if(play_stop){
		rampCount--;
	}
}


void init_dtmf(){
	DDRB = 0;
	DDRB |= OUTPUT_PIN;
	int i;
	for (i=0; i<256; i++){
		sineTable[i] = (char)(127.0 * sin(6.283*((float)i)/256.0)) ;
		rampTable[i] = i>>1 ;
	}

	TCCR0B = 1;
	TIMSK0 = 0;
	TIMSK0 |= TIMER0_OVERFLOW_INTERRUPT_ENABLE;
	//turn on fast PWM and OC0A - output 
	TCCR0A = 0;
	TCCR0A = (1<<COM0A0) | (1<<COM0A1) | (1<<WGM00) | (1<<WGM01) ; 

	OCR0A = 128;
	play_start = FALSE;
	play_stop = FALSE;
	rampCount = 0;
	sample = 0;

	sei();
}

//sets appropriate DDS increment values
//based on request frequencies.
void play(int fA, int fB){
	is_playing = TRUE;
	switch( fA){
		case 1209: increment_a = INCREMENT_1209; break;
		case 1336: increment_a = INCREMENT_1336; break;
		case 1477: increment_a = INCREMENT_1477; break;
		case 697:  increment_a = INCREMENT_697; break;
		case 941: increment_a = INCREMENT_941; break;
		default: increment_a = 0; break;
	}

	switch ( fB) {
		case 697: increment_b = INCREMENT_697; break;
		case 770: increment_b = INCREMENT_770; break;
		case 852: increment_b = INCREMENT_852; break;
		case 941: increment_b = INCREMENT_941; break;
		case 1209: increment_a = INCREMENT_1209; break;
		default: increment_b = 0; break;
	}
}

void timed_play(int fA, int fB, int duration){
	dds_duration = duration;
	play(fA, fB);
	is_playing=TRUE;
}

void stop_playing(){
	is_playing = FALSE;
	increment_a = increment_b = 0;
	accumulator_a = accumulator_b = 0;
}


void update_status_variables(){
	if(changed == TRUE ){
		changed = FALSE;
		
		if (is_playing) {
			if (PushState == Pushed && rampCount < 255) {
				rampCount++;
			}
			if (PushState == NoPush && rampCount > 0 ) {
				rampCount--;
			}
		}
		
	}
}

