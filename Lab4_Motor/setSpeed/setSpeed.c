#define F_CPU 16000000UL

#define UART_BAUD 9600
/*
Tasks = read motor speed
	  = set motor speed with PWM
	  = PID control loop 
*/
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

//#include "uart.h"
//#include "atmega1284p.h"
#include <inttypes.h>

#define OUTPUT_PIN (1 << PINB3);

#define T0_CS00 1
#define TIMER0_OVERFLOW_INTERRUPT_ENABLE (1 << TOIE0)

#define COMPARE_MATCH_OUTPUT_A0 (1 << COM0A0)
#define COMPARE_MATCH_OUTPUT_A1 (1 << COM0A1)

#define WAVE_GEN_M00 (1<<WGM00)
#define WAVE_GEN_M01 (1 << WGM01)

void init_pwm(){
	DDRB = 0;
	DDRB |= OUTPUT_PIN;

	TIMSK0 = 0;
	TIMSK0 |= TIMER0_OVERFLOW_INTERRUPT_ENABLE;
	//turn on fast PWM and OC0A - output 
	TCCR0A = 0;
	//divide PWM clock by 1024 
	TCCR0B = (1 << CS02)  |( 1 << CS00 )  ;
	TCCR0A = (1<<COM0A0) | (1<<COM0A1) | (1 << WGM02) | (1<<WGM00) | (1<<WGM01) ; 

	OCR0A = 1;
	

	sei();
}

int main(void){
	init_dtmf();
}
