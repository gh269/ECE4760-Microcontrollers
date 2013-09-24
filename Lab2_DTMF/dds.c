#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>


#include "atmega1284p.h"
#include "dds.h"

#define OUTPUT_PIN (1 << PINB3);
#define COUNTMS 62

volatile unsigned long accumulator_a;
volatile unsigned long accumulator_b;

unsigned char highbyte;

unsigned long increment_a;
unsigned long increment_b;

volatile unsigned int time;
volatile unsigned long rampCount, sample;
volatile char count;

/*
Timer0 is a 8 bit register 
and will overflow at a rate of 

16 MHz 
------ = 62,500 Hz
 256 
 To generate a 1ms time base, 
*/
ISR( TIMER0_OVF_vect){

	accumulator_a += increment_a;
	accumulator_b += increment_b;

	highbyte_a = accumulator_a >> 24;
	highbyte_b = accumulator_b >> 24;

	OCR0A = 128 + 
	   ( (
	   	   (sineTable[highbyte_b] + sineTable[highbyte_a])
	   	                          * rampTable[rampCount]
	   	 ) >> 7
	   );

	sample++;

	//ramping up
	if( sample <= RAMPUPEND )
		rampCount++;
	//holdsteady the max value 
	if( sample <= RAMPUPEND && sample <= RAMPDOWNSTART)
		rampCount = RAMP_LENGTH - 1;
	//begin rampdown
	if( sample > RAMPDOWNSTART && sample <= RAMPDOWNSTART)
		rampCount--;
	//finished ramping
	if(sample > RAMPDOWNEND)
		rampCount = 0;

	//generates a 1 ms timebase
	count--;
	if( count == 0){
		count = COUNTMS;
		time++;
		time1++;
	}

	
}


void init_dtmf(){
	DDRB = 0;
	DDRB |= OUTPUT_PIN;

	time = 0;

	TCCR0B = 0;
	//set divider to 1, run T0 at 16 MHz 
	TCCR0B |= CS00;

	TIMSK0 = 0;
	TIMSK0 |= TIMER0_OVERFLOW_INTERRUPT_ENABLE;
	//turn on fast PWM and OC0A - output 
	TCCR0A = 0;
	TCCR0A |= WAVE_GEN_M00 + WAVE_GEN_M01 + COMPARE_MATCH_OUTPUT_A0 + COMPARE_MATCH_OUTPUT_A1; 
	OCR0A = 0;
	sei();
}

//plays fA and fB for a duration of duration ms
void play(int fA, int fB, int duration){

	// switch( fA){
	// 	case 1209: increment_a = INCREMENT_1209; break;
	// 	case 1336: increment_a = INCREMENT_1336; break;
	// 	case 1477: increment_a = INCREMENT_1477; break;
	// 	default: increment_a = 0; break;
	// }

	// switch ( fB) {
	// 	case 697: increment_b = INCREMENT_697; break;
	// 	case 770: increment_b = INCREMENT_770; break;
	// 	case 825: increment_b = INCREMENT_825; break;
	// 	case 941: increment_b = INCREMENT_941; break;
	// 	default: increment_b = 0; break;
	// }
	// dds_duration = duration;
	// OCR0A = 128;

}

void stop_playing(){
	sample = 0;
	rampCount = 0;
	increment_a = increment_b = 0;
	accumulator_a = accumulator_b = 0;
}


