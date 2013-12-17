#define F_CPU 16000000UL
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/eeprom.h>

#include "uart.h"


//UART File Desc
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);


#define t1 250
#define begin {
#define end } 

void task1(void); //
void initialize(void);

//timeout counter for task1
volatile unsigned char time1;
// timer 1 capture variables for computing sq wave period	
volatile unsigned int T1capture, lastT1capture, period ; 
// polling variables for computing sq wave period (low accuracy)
unsigned int periodPoll, T1poll, lastT1poll; 

char ACObit, lastACObit;

//timer 0 overflow vector
//triggers on TCNT0= OCR0A
//TCNT0 is a 16 bit register
ISR (TIMER0_COMPA_vect){
	//define the task1 time counter
	if( time1>0)
		--time1;
}


ISR (TIMER1_CAPT_vect){
	//read timer1 input capture register
	//The Input Capture Register ICR1 is a 16 bit register used to record the value of TCNT1 when an external event happens
	T1capture = ICR1;
	period = T1capture - lastT1capture;
	lastT1capture = T1capture;
}

void initialize(void){

	//timer0 1ms time base
	//Timer 0 increments a counter
	//Timer Interrupt Mask Register
	//Output Capture/Compare A interrupt enable
	TIMSK0 = (1 << OCIE0A);
	//OCR0A is the period register 
	OCR0A = 249; 
	//set prescalar to dividy by 64
	// 16 MHz / 64 = 250 KHz / OCR0A = 1 KHz = 1 msec
	//0b00001011
	TCCR0B = 3; 
	//turn on clear-on-match - timer A ISR will clear TCNT0 on match
	TCCR0A = (1<<WGM01);
	//setup timer1 for full speed and capture an edge on analog comparator pin B.3
	//capture on positive edge, full counting rate
	// ICES1 - 1: rising edge 
	//		 - 0: falling edge
	//CS = 001 - > no prescaling
	//period of Timer1.B = 1/16 us
	//on capture, ICR1 has the value 
	TCCR1B = (1 << ICES1) + 1;

	//turn on Timer1 interrupt on capture
	TIMSK1 = (1 << ICIE1);

	//set analog comp to connect to timer capture input
	//turn on bandgap reference on the positie iinput
	//Analog Comparator Control and Status Register
	//0b01000100  ;
	//ACBG - Analog Comparator Bandgap Select - use fixed voltage 
	//     that is, use HIGH as reference voltage
	//ACIC - connects timer1's input capture to the Analog Comparator
	ACSR = (1 << ACBG) | (1 <<ACIC);
	//clear port B Data Direction to Input
	DDRB = 0;

	//setup timer2 for square wave generation with no ISR
	//200 cycle period from OC2
	OCR2A = 99; //100 cycles/ half-period
	// CS - 001 -> no prescaling
	TCCR2B = 1;
	//toggle on OC2A on Compare Match (COM2A0)
	//set to PWM Phase Correct mode
	TCCR2A = (1 << COM2A0) | (1<<WGM21);
	//Port D.7 is OC2A
	DDRD = (1 << PIND7);

	time1 = t1;
	uart_init();
	stdout = stdin = stderr = &uart_str;
	fprintf(stdout, "Starting timers...\n\r");

	//enable interrupts()
	sei();
}