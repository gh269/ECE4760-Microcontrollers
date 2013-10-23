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
/*
Use Timer 2 to measure the rotation time and output it onto the UART

use int0_vect 
*/
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
char flag; 

void initialize_external_interrupt(void){
	EIMSK = 1 << INT0; //turn on int0
	EICRA = 3;		   //rising edge
	TCCR2B = 7;		   // divide by 1024
	TIMSK2 = 1;        //turn on timer2 overflow ISR for double 
					   //precision time
}
double cycles_to_rpm(int cycles, int frequency){
	return 60 * ( (double) cycles ) / ( (double) frequency );
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

x cycles *  1 second       [ s] 
			--------- = 
			15,625 cycles

*/
ISR( INT0_vect){
	motor_period = TCNT2 + motor_period_ovlf;
	TCNT2 = 0;
	motor_period_ovlf = 0;
	flag = FALSE;
}


ISR(TIMER2_OVF_vect){
	motor_period_ovlf = motor_period_ovlf + 256;
}

int main(void){
	initialize_external_interrupt();
	uart_init();
	stdout = stdin = stderr = &uart_str;
  	fprintf(stdout, "Starting...\n\r");
	sei();
	while(1){
		if( flag ) {
			fprintf(stdout, "RPM: %f\n", cycles_to_rpm(motor_period));
			flag = TRUE;
		}
	}
}