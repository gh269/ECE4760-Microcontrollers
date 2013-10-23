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
double cycles_to_rpm(int cycles, int frequency){
	return  (   (60*((double)frequency)) / ((double) cycles * 7) );
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
int main(void){
	initialize_external_interrupt();



	uart_init();
	stdout = stdin = stderr = &uart_str;
  	fprintf(stdout, "Starting...\n\r");
	sei();
	while(1){
		if( flag ) {
			//fprintf(stdout, "%d\n", cou);
			//cou = (cou+1) % 256;
			fprintf(stdout, "RPM: %f\n", cycles_to_rpm(motor_period, 15625));
			flag = FALSE;
		}
	}
}
