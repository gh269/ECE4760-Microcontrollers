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


#define RAMP_LENGTH 312

signed char sineTable[256];
char rampTable[256]; 
/*
Timer0 is a 8 bit register 
and will overflow at a rate of 

16 MHz 
------ = 62,500 Hz
 256 
 To generate a 1ms time base, 
*/

volatile unsigned long accumulator ;


volatile unsigned long increment;
ISR( TIMER0_OVF_vect){

//	highbyte_a = (char) (accumulator_a >> 24);7
//	highbyte_b = (char) (accumulator_b >> 24);
	//changed = TRUE;


	//accumulator = accumulator + increment ;
	//highbyte = (char)(accumulator >> 24) ;
	
	// output the wavefrom sample
	//OCR0A = 128 + ((sineTable[highbyte] * rampTable[rampCount])>>7) ;
	changed = TRUE;


	OCR0A = 128 + 
	   ( (
	   	   (sineTable[(accumulator_a >> 24)] + sineTable[(accumulator_b >> 24)]) * rampTable[rampCount]
	   	 ) >> 8
	   );
	accumulator_a += increment_a;
	accumulator_b += increment_b;
	//generates a 1 ms timebase
	count--;	
}


void init_dtmf(){
	DDRB = 0;
	DDRB |= OUTPUT_PIN;
	int i;
	for (i=0; i<256; i++){
		sineTable[i] = (char)(127.0 * sin(6.283*((float)i)/256.0)) ;
		// the following table needs 
		// rampTable[0]=0 and rampTable[255]=127
		rampTable[i] = i>>1 ;
	}
	time = 0;

	// TCCR0B = 0;
	// //set divider to 1, run T0 at 16 MHz 
	// TCCR0B |= T0_CS00;
	TCCR0B = 1;
	TIMSK0 = 0;
	TIMSK0 |= TIMER0_OVERFLOW_INTERRUPT_ENABLE;
	//turn on fast PWM and OC0A - output 
	TCCR0A = 0;
	TCCR0A = (1<<COM0A0) | (1<<COM0A1) | (1<<WGM00) | (1<<WGM01) ; 

	//TCCR0A |= WAVE_GEN_M00 + WAVE_GEN_M01 + COMPARE_MATCH_OUTPUT_A0 + COMPARE_MATCH_OUTPUT_A1; 
	OCR0A = 128;
	//increment = INCREMENT_941;
	//increment_a = INCREMENT_941;
	//increment_b = INCREMENT_941;
	sei();
}

//plays fA and fB for a duration of duration ms
void play(int fA, int fB){
	//if( !is_playing ){
		//is_playing = TRUE;
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
			case 825: increment_b = INCREMENT_825; break;
			case 941: increment_b = INCREMENT_941; break;
			case 1209: increment_a = INCREMENT_1209; break;
			default: increment_b = 0; break;
		}
		//dds_duration = duration;
		//OCR0A = 128;	
	//}


}

void stop_playing(){
	is_playing = FALSE;
	sample = 0;
	rampCount = 0;
	increment_a = increment_b = 0;
	accumulator_a = accumulator_b = 0;
}

void update_status_variables(){
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
/*
int main(){
	init_dtmf();
	play(1477, 697, 1000);
	while(1){
		if( count <= 0){
			count = COUNTMS;
			//time++;
			//time1++;
		}

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
*/
