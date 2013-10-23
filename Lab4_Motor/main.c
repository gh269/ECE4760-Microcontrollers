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
volatile double prop_gain;
volatile double diff_gain;
volatile double int_gain; 

// motor variables
volatile int motor_period;
volatile int motor_period_ovlf;

// lcd variables
const int8_t LCD_initialize[] PROGMEM = "RPM: ";
const int8_t LCD_number[] PROGMEM = "Number=\0";
int8_t lcd_buffer[17];	// LCD display buffer

/********************************************************************/
// 							ISRs & Helper Functions
/********************************************************************/
// --- external interrupt ISR ------------------------
ISR (INT0_vect) {
        motor_period = TCNT2 + motor_period_ovlf  ;
        TCNT2 = 0 ;
        motor_period_ovlf = 0 ;
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
	CopyStringtoLCD(LCD_initialize, 0, 0);
}

//********************************************************** 
//Set it all up
void initialize(void) {
  DDRC = 0xff;    	// led connections
  PORTC = 0x00;
  DDRB = 0x00; 		// switch connections
  PORTB = 0xff; 	// pullup on

  //******************** 
  //initialize variables
  trtWait(SEM_SHARED) ;
  speed = 0;
  prop_gain = 0;
  diff_gain = 0;
  int_gain = 0; 
  trtSignal(SEM_SHARED) ;

  //******************** 
  //init LCD
  init_lcd();
  LCDclr();
  // put some stuff on LCD
  CopyStringtoLCD(LCD_number, 0, 0);//start at char=0 line=0
 
  //******************** 
  //set up INT0
  EIMSK = (1 << INT0) ; // turn on int0
  EICRA = 3 ;       // rising edge
  // turn on timer 2 to be read in int0 ISR
  TCCR2B = 7 ; // divide by 1024
  // turn on timer 2 overflow ISR for double precision time
  TIMSK2 = 1 ;
  
  //********************
  //crank up the ISRs
  sei();
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
		if (cmd[0] == 's') {
			speed = num;
			fprintf(stdout, "%d\n\r", speed);
		}
		if (cmd[0] == 'p')		
			prop_gain = (int) num;
		if (cmd[0] == 'i')		
			diff_gain = (int) num;
		if (cmd[0] == 'i')		
			diff_gain = num;

		/*
		switch (cmd[0]) {
			case 's':	
				speed = (int) num;
				fprintf(stdout, "%u\n\r", speed);
				break;
			case 'p':
				prop_gain = (int) num;
				fprintf(stdout, "%u\n\r", prop_gain);
				break;
			case 'i':
				diff_gain = (int) num;
				fprintf(stdout, "%u\n\r", diff_gain);
				break;
			case 'd': 
				int_gain = num;
				fprintf(stdout, "%u\n\r", int_gain);
				break;
			default:
				break;
		}
		*/
		trtSignal(SEM_SHARED);
	}
}

// --- define task 2  ----------------------------------------
void lcdComm(void* args) {
	uint32_t rel, dead;
	// increment time counter and format string 
	while (TRUE) {
	  trtWait(SEM_SHARED) ;
	  //fprintf(stdout, "%u\n\r", speed);
	  //lcd_buffer = NULL;
	  //int8_t lcd_buffer[17]
	  //lcd_buffer = (int8_t *) (calloc(17, sizeof(int8_t)));
	  //int lcd_buf_i;
	  //for( lcd_buf_i = 0; lcd_buf_i < 17; lcd_buf_i++){
	  //	lcd_buffer[lcd_buf_i] ='\0';
	  //}

	  sprintf(lcd_buffer, "%d    ", speed);
	  //fprintf(stdout, "%s\n\r", lcd_buffer);
	  trtSignal(SEM_SHARED) ;	                 
	  LCDGotoXY(7, 0);
	  // display the count 
	  LCDstring(lcd_buffer, strlen(lcd_buffer));
	  rel = trtCurrentTime() + SECONDS2TICKS(0.2);
	  dead = trtCurrentTime() + SECONDS2TICKS(0.4);
	  trtSleepUntil(rel, dead);	
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
