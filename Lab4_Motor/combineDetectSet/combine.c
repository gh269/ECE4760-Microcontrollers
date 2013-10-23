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
#include "pid.h"
#include "uart.h"
#include "atmega1284p.h"
#include <inttypes.h>

volatile int16_t motor_period_ovlf = 0;
volatile int16_t motor_period = 0; 

#define OUTPUT_PIN (1 << PINB3);

#define T0_CS00 1
#define TIMER0_OVERFLOW_INTERRUPT_ENABLE (1 << TOIE0)

#define COMPARE_MATCH_OUTPUT_A0 (1 << COM0A0)
#define COMPARE_MATCH_OUTPUT_A1 (1 << COM0A1)

#define WAVE_GEN_M00 (1<<WGM00)
#define WAVE_GEN_M01 (1 << WGM01)

double K_P = 1;

double K_I = 0.1;
double K_D = 0.01;
char pid_ready = FALSE;
struct PID_DATA pid_data;

//Target [ T2 Cycles ] @ 15625 Hz
volatile double target; 
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
	return ( (60*((double)frequency)) / ((double) rpm * 7) );
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
/*
PID is run once every 50 seconds
			16 000 000 
F_T2 =   --------------- = 15, 625 Hz
			   1024

			   15, 625 Hz
F_T2_OVF =   ----------------   =  61.035 Hz
				  256
CLOSE ENOUGH
*/
ISR (TIMER2_OVF_vect) {
        motor_period_ovlf = motor_period_ovlf + 256 ;
        pid_ready = TRUE;
}	

//set control input to the system
//sets the output from the controller 
// as input to the system
void set_input( int8_t input_value){
	OCR0A = input_value;
}

//motor period [ T2 cycles] @ 15625 Hz
int16_t get_measurement(void){
	return motor_period;
}


int16_t get_reference(void){
	return target;
}


void init_pwm(){
	DDRB = 0;
	DDRB |= OUTPUT_PIN;

	TIMSK0 = 0;
	//TIMSK0 |= TIMER0_OVERFLOW_INTERRUPT_ENABLE;
	//turn on fast PWM and OC0A - output 
	TCCR0A = 0;
	//divide PWM clock by 1024 
	TCCR0B = 3;// (1 << CS01)  |( 1 << CS00 )  ;
	TCCR0A = (1<<COM0A0) | (1<<COM0A1) | (1 << WGM02) | (1<<WGM00) | (1<<WGM01) ; 

	OCR0A =87;
	//sei();
}
int main(void){
	initialize_external_interrupt();
	init_pwm();

	int16_t referenceValue,inputValue;
	double measurementValue = 0;
	double rpm = 0;
	double motor_period_prime = 0;
	inputValue = 0;
	//pid_data = (struct PID_DATA *) malloc(sizeof(pidData_t));
	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pid_data);

	target = rpm_to_cycles(2000, 15625);
	uart_init();
	stdout = stdin = stderr = &uart_str;
  	fprintf(stdout, "Starting...\n\r");
	sei();
	while(1){

		if( flag ) {
			//fprintf(stdout, "%d\n", cou);
			//cou = (cou+1) % 256;
			rpm = cycles_to_rpm(motor_period, 15625);
			//motor_period_prime = rpm_to_cycles(rpm, 15625);
			//fprintf(stdout, "RPM: %f\n, Cycles: %f ", rpm, motor_period_prime);
			flag = FALSE;
			fprintf(stdout, "RPM: %f\n", rpm);
			
			if (pid_ready){
				pid_ready = FALSE;
				referenceValue = cycles_to_rpm(get_reference(), 15625);
				measurementValue = cycles_to_rpm(motor_period, 15625);
				inputValue = pid_Controller(referenceValue, motor_period, & pid_data);
			}
			
			fprintf(stdout, "RPM: %f,  input: %d, reference: %d, measurement: %f\n\r ", 
					rpm,  inputValue, referenceValue, measurementValue);
			
		}
	}
}
