#include "trtSettings.h"
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>

#include "trtUart.h"
#include "trtkernel_1284.c"
#include "trtUART.c"
#include "lcd_lib.c"

/********************************************************************/
//						UART Functions
/********************************************************************/
FILE uart0 = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

/********************************************************************/
//						FSM Variables
/********************************************************************/
// semaphore to protect shared variable
#define SEM_SHARED 4

// input arguments to each thread
// not actually used in this example
int args[2] ;

// binary states
#define FALSE 0
#define TRUE 1

// input variables
volatile double speed;
volatile double real_speed;
volatile double prop_gain;
volatile double diff_gain;
volatile double int_gain; 

// motor variables
#define OUTPUT_PIN (1 << PINB3);
#define T0_CS00 1
#define TIMER0_OVERFLOW_INTERRUPT_ENABLE (1 << TOIE0)
#define COMPARE_MATCH_OUTPUT_A0 (1 << COM0A0)
#define COMPARE_MATCH_OUTPUT_A1 (1 << COM0A1)
#define WAVE_GEN_M00 (1<<WGM00)
#define WAVE_GEN_M01 (1 << WGM01)
volatile int motor_period;
volatile int motor_period_ovlf;
volatile char flag = 0; 

// lcd variables
const int8_t LCD_line1[] PROGMEM = "iRPM=\0";
const int8_t LCD_line2[] PROGMEM = "rRPM=\0";
int8_t lcd_buffer[17];	// LCD display buffer

/********************************************************************/
// 							ISRs & Helper Functions
/********************************************************************/
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

//**********************************************************
// LCD setup
void init_lcd(void) {
	LCDinit();	//initialize the display
	LCDcursorOFF();
	LCDclr();				//clear the display
	LCDGotoXY(0,0);
}

//**********************************************************
// pwm setup
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
}

//********************************************************** 
//Set it all up
void initialize(void) {
  DDRC = 0xff;    	// led connections
  PORTC = 0x00;
  //DDRB = 0x00; 		// switch connections
  //PORTB = 0xff; 	// pullup on

  //******************** 
  //initialize variables
  trtWait(SEM_SHARED) ;
  speed = 0;
  real_speed = 0;
  prop_gain = 0;
  diff_gain = 0;
  int_gain = 0; 
  trtSignal(SEM_SHARED) ;

  //******************** 
  //init LCD
  init_lcd();
  LCDclr();
  // put some stuff on LCD
  CopyStringtoLCD(LCD_line1, 0, 0);//start at char=0 line=0
  CopyStringtoLCD(LCD_line2, 0, 1); 
  //******************** 
  //set up INT0
  EIMSK = (1 << INT0) ; // turn on int0
  EICRA = 3 ;       // rising edge
  // turn on timer 2 to be read in int0 ISR
  TCCR2B = 7 ; // divide by 1024
  // turn on timer 2 overflow ISR for double precision time
  TIMSK2 = 1 ;
  
  //********************
  //set up PWM
  init_pwm();

  //********************
  //crank up the ISRs
  sei();
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

double rpm_to_cycles(int rpm, int frequency){
	return ( (60*((double)frequency)) / ((double) rpm * 7) )
}
//==================================================

/********************************************************************/
// 							FSM Tasks
/********************************************************************/
// --- define task 1  ----------------------------------------
void serialComm(void* args) {
	// Declare the command and num variables
	volatile double num ;
	char cmd[4] ;
    // initialize
    initialize();
	while (TRUE) {
		// commands:
		// 's' sets the motor speed
		// 'p' sets the proportional gain
		// 'i' sets the differential gain
		// 'd' sets the integral gain
		fprintf(stdout, ">") ;
		fscanf(stdin, "%s%d", cmd, &num) ;
		//fprintf(stdout, "%s%le\n\r", cmd, &num);
		// update shared variables
		trtWait(SEM_SHARED) ;
		if (cmd[0] == 's')
			speed = num;
		if (cmd[0] == 'p')		
			prop_gain = num;
		if (cmd[0] == 'i')		
			diff_gain = num;
		if (cmd[0] == 'i')		
			diff_gain = num;
		trtSignal(SEM_SHARED);
	}
}

// --- define task 2  ----------------------------------------
void lcdComm(void* args) {
	uint32_t rel, dead;
	// increment time counter and format string 
	while (TRUE) {
	  // display the ideal count
	  trtWait(SEM_SHARED) ;
	  sprintf(lcd_buffer, "%d   ", speed);
	  LCDGotoXY(7, 0);
	  LCDstring(lcd_buffer, strlen(lcd_buffer));
	  // display the actual count 
	  sprintf(lcd_buffer, "%d   ", real_speed);
	  LCDGotoXY(7, 1);
	  LCDstring(lcd_buffer, strlen(lcd_buffer));
	  trtSignal(SEM_SHARED) ;
	  // sleep
	  rel = trtCurrentTime() + SECONDS2TICKS(0.2);
	  dead = trtCurrentTime() + SECONDS2TICKS(0.4);
	  trtSleepUntil(rel, dead);	
  	}
}

// --- define task 3  ----------------------------------------
void adjustSpeed(void* args) {
	while(1){
		// detection of the fan speed
		if( flag ) {
			trtWait(SEM_SHARED);
			real_speed = cycles_to_rpm(motor_period, 15625);
			trtSignal(SEM_SHARED);
			//fprintf(stdout, "RPM: %f\n", real_speed);
			flag = FALSE;

			//INSERT PID SHIT HERE



		}
	}

}

// --- Main Program ----------------------------------
int main(void) {
  //init the UART -- trt_uart_init() is in trtUart.c
  trt_uart_init();
  stdout = stdin = stderr = &uart0;
  fprintf(stdout,"\n\r TRT 9feb2009\n\r\n\r");
    // start TRT
  trtInitKernel(80); // 80 bytes for the idle task stack

  // --- create semaphores ----------
  // You must create the first two semaphores if you use the uart
  trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; // uart receive ISR semaphore
  trtCreateSemaphore(SEM_STRING_DONE,0) ;  // user typed <enter>

  // variable protection
  trtCreateSemaphore(SEM_SHARED, 1) ; // protect shared variable

  // --- create tasks  ----------------
  trtCreateTask(serialComm, 1000, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[0]));
  trtCreateTask(lcdComm, 1000, SECONDS2TICKS(0.2), SECONDS2TICKS(0.4), &(args[0]));
  trtCreateTask(adjustSpeed, 1000, SECONDS2TICKS(0.02), SECONDS2TICKS(0.02), &(args[0]));

  // --- Idle task --------------------------------------
  // For debugging, blink an LED
  // For production, you would probably comment out the body of the WHILE
  while (1) {
  /*
  	PORTC = PORTC ^ 0x01 ;
  	_delay_ms(500) ;	
  */
  }

} // main
