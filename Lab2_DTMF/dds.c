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

volatile unsigned int rampCount, sample;

#define RAMP_LENGTH 312

signed char sineTable[256];
/* = 

{ 0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 
39, 42, 45, 48, 51, 54, 57, 59, 62, 65, 67, 70, 
73, 75, 78, 80, 82, 85, 87, 89, 91, 94, 96, 98, 
100, 102, 103, 105, 107, 108, 110, 112, 113, 114, 116, 117, 
118, 119, 120, 121, 122, 123, 123, 124, 125, 125, 126, 126, 
126, 126, 126, 126, 126, 126, 126, 126, 126, 125, 125, 124, 
123, 123, 122, 121, 120, 119, 118, 117, 116, 114, 113, 112, 
110, 108, 107, 105, 103, 102, 100, 98, 96, 94, 91, 89, 
87, 85, 82, 80, 78, 75, 73, 70, 67, 65, 62, 59, 
57, 54, 51, 48, 45, 42, 39, 36, 33, 30, 27, 24, 
21, 18, 15, 12, 9, 6, 3, 0, -3, -6, -9, -12, 
-15, -18, -21, -24, -27, -30, -33, -36, -39, -42, -45, -48, 
-51, -54, -57, -59, -62, -65, -67, -70, -73, -75, -78, -80, 
-82, -85, -87, -89, -91, -94, -96, -98, -100, -101, -103, -105, 
-107, -108, -110, -111, -113, -114, -116, -117, -118, -119, -120, -121, 
-122, -123, -123, -124, -125, -125, -126, -126, -126, -126, -126, -126, 
-126, -126, -126, -126, -126, -125, -125, -124, -123, -123, -122, -121, 
-120, -119, -118, -117, -116, -114, -113, -112, -110, -108, -107, -105, 
-103, -102, -100, -98, -96, -94, -91, -89, -87, -85, -82, -80, 
-78, -75, -73, -70, -67, -65, -62, -59, -57, -54, -51, -48, 
-45, -42, -39, -36, -33, -30, -27, -24, -21, -18, -15, -12, 
-9, -6, -3  };
*/
char rampTable[256]; 
/*= 

{ 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 
5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 9, 
10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 
15, 15, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 
20, 20, 20, 21, 21, 22, 22, 22, 23, 23, 24, 24, 
24, 25, 25, 26, 26, 26, 27, 27, 28, 28, 28, 29, 
29, 30, 30, 31, 31, 31, 32, 32, 33, 33, 33, 34, 
34, 35, 35, 35, 36, 36, 37, 37, 37, 38, 38, 39, 
39, 40, 40, 40, 41, 41, 42, 42, 42, 43, 43, 44, 
44, 44, 45, 45, 46, 46, 46, 47, 47, 48, 48, 49, 
49, 49, 50, 50, 51, 51, 51, 52, 52, 53, 53, 53, 
54, 54, 55, 55, 55, 56, 56, 57, 57, 57, 58, 58, 
59, 59, 60, 60, 60, 61, 61, 62, 62, 62, 63, 63, 
64, 64, 64, 65, 65, 66, 66, 66, 67, 67, 68, 68, 
69, 69, 69, 70, 70, 71, 71, 71, 72, 72, 73, 73, 
73, 74, 74, 75, 75, 75, 76, 76, 77, 77, 77, 78, 
78, 79, 79, 80, 80, 80, 81, 81, 82, 82, 82, 83, 
83, 84, 84, 84, 85, 85, 86, 86, 86, 87, 87, 88, 
88, 89, 89, 89, 90, 90, 91, 91, 91, 92, 92, 93, 
93, 93, 94, 94, 95, 95, 95, 96, 96, 97, 97, 98, 
98, 98, 99, 99, 100, 100, 100, 101, 101, 102, 102, 102, 
103, 103, 104, 104, 104, 105, 105, 106, 106, 106, 107, 107, 
108, 108, 109, 109, 109, 110, 110, 111, 111, 111, 112, 112, 
113, 113, 113, 114, 114, 115, 115, 115, 116, 116, 117, 117, 
118, 118, 118, 119, 119, 120, 120, 120, 121, 121, 122, 122, 
122, 123, 123, 124, 124, 124, 125, 125, 126, 126, 127  };
*/
/*
Timer0 is a 8 bit register 
and will overflow at a rate of 

16 MHz 
------ = 62,500 Hz
 256 
 To generate a 1ms time base, 
*/

volatile unsigned long accumulator ;
volatile unsigned char changed;


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
