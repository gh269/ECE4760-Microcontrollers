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
#include "atmega1284p.h"
#include <inttypes.h>

#define K_P 10
#define K_I 2
#define K_D 5

struct GLOBAL_FLAGS{
	uint8_t pidTimer:1;
	uint8_t dummy:7;
} gFlags = {0, 0};

int target = 0;
struct PID_DATA pidData;

int16_t Get_Reference(void){
	return target;
}
int16_t Get_Measurement(void){
	return 4;
}


#pragma vector=TIMER0_OVF_vect
__interrupt void TIMER0_OVF_IST(void){
	static uint16_t i = 0;
	if( i < TIMER_INTERVAL)
		i++;
	else{
		gFlags.pidTimer = TRUE;
		i = 0;
	}
}

//set control input to system
//sets the out from pthe controler as input to the system
void Set_Input(1nt16_t inputValue){
	target = inputValue;
}


//Time Interval = ( desired inteval [ sec] * f_T2 ) / 255
//Handled on OVF
//
#define TIME_INTERVAL 157

void main(void){
	int16_t referenceValue, measurementValue, inputValue;
	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);

	if(gFlags.pidTimer){
		referenceValue = Get_Reference();
		measurementValue = Get_Measurement();
		inputValue = 

		gFlags.pidTimer = FALSE;
	}

}