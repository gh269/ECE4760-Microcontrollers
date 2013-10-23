#define F_CPU 16000000UL

#define UART_BAUD 9600
/*
Tasks = read motor speed
	  = set motor speed with PWM
*/
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart.h"
#include "atmega1284p.h"
#include <inttypes.h>

volatile int motor_period_ovlf = 0;
volatile int motor_period = 0;

#define OUTPUT_PIN (1 << PINB3);

#define T0_CS00 1
#define TIMER0_OVERFLOW_INTERRUPT_ENABLE (1 << TOIE0)

#define COMPARE_MATCH_OUTPUT_A0 (1 << COM0A0)
#define COMPARE_MATCH_OUTPUT_A1 (1 << COM0A1)

#define WAVE_GEN_M00 (1<<WGM00)
#define WAVE_GEN_M01 (1 << WGM01)



/*
Use Timer 2 to measure the rotation time and output it onto the UART

use int0_vect 
*/
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
volatile char flag= 0; 
volatile int cou = 0;
void initialize_external_interrupt(void){
	//set up INT0
	EIMSK = 1<<INT0 ; // turn on int0
	EICRA = 3 ;       // rising edge
	// turn on timer 2 to be read in int0 ISR
	TCCR2B = 7 ; // divide by 1024
	// turn on timer 2 overflow ISR for double precision time
	TIMSK2 = 1 ;
}
/*

motor period cycles       1 second        1 minute         7
-------------------   *   ---------  *   ----------    * -------------- 
    		            15,625 cycles     60 seconds      1 rotation

*/
double cycles_to_rpm(int cycles, int frequency){
	return  (   (60*((double)frequency)) / ((double) cycles * 7) );
}
/*

rpm rotations      1 minute 		     7		    1 second
-------------  * --------------  *  ---------- * ---------------- 
	1 minute       60 seconds       1 rotation    15, 625 cycles 

same conversion factor?? WTH?
*/

double rpm_to_cycles(double rpm, int frequency){
	return ( (60*((double)frequency)) / ((double) rpm * 7) )
}
//----external ISR------
/*			16 000 000 
F_T2 =   --------------- = 15, 625 Hz
			   1024
TCNT2 = t % 256
@t=0, ^ motor_period = TCNT2 + motor_period_ovlf 
T = 256;
@t=256 ticks, ^ MP = 255 + 0;
T < 256;
@t=y ticks, ^ MP = y + 0;
@t > 256;
mp_ovlf = 256;

motor period cycles       1 second        1 minute         7
-------------------   *   ---------  *   ----------    * -------------- 
    		            15,625 cycles     60 seconds      1 rotation

*/
// --- external interrupt ISR ------------------------
ISR (INT0_vect) {
        motor_period = TCNT2 + motor_period_ovlf  ;
        TCNT2 = 0 ;
        motor_period_ovlf = 0 ;
		flag = TRUE;
		
}
// --- set up extra 8 bits on timer 2 ----------------
ISR (TIMER2_OVF_vect) {
        motor_period_ovlf = motor_period_ovlf + 256 ;
}

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
	initialize_external_interrupt();
	init_pwm();


	uart_init();
	stdout = stdin = stderr = &uart_str;
  	fprintf(stdout, "Starting...\n\r");
	sei();
	while(1){
		if( flag ) {
			//fprintf(stdout, "%d\n", cou);
			//cou = (cou+1) % 256;
			double rpm = cycles_to_rpm(motor_period, 15625);
			double motor_period_prime = rpm_to_cycle(rpm, 15625);
			fprintf(stdout, "RPM: %f\n, Cycles: ", rpm, motor_period_prime);
			flag = FALSE;
		}
	}
}
