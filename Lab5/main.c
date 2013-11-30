#include "trtSettings.h"
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>

#include "trtkernel_1284.c"
#include "trtUart.h"
#include "trtUart.c"
#include "lcd_lib.h"
#include "lcd_lib.c"

bleh


/********************************************************************/
//						UART Functions
/********************************************************************/
FILE uart0 = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

/********************************************************************/
//						FSM Variables
/********************************************************************/
// semaphore to protect shared variable
// semaphores 1-3 are used by the UART
#define SEM_SHARED 4
// input arguments to each thread
// not actually used in this example
int args[2] ;

// binary states
#define FALSE 0
#define TRUE 1

// input and output variables
volatile int cTemp;		// current temperature
volatile int dTemp;		// desired temperature
volatile int time_rem;  // time remaining in seconds


/*
// input variables
volatile double speed;
volatile double real_speed;
volatile double prop_gain;
volatile double diff_gain;
volatile double int_gain; 

// motor variables
#define OUTPUT_PIN (1 << PINB3);
#define OUTPUT_PIN2 (1 << PINB4);
#define T0_CS00 1
#define TIMER2_OVERFLOW_INTERRUPT_ENABLE (1 << TOIE2)
#define COMPARE_MATCH_OUTPUT_A0 (1 << COM0A0)
#define COMPARE_MATCH_OUTPUT_A1 (1 << COM0A1)
volatile int motor_period;
volatile int motor_period_ovlf;
volatile char flag = 0; 
char pid_ready = FALSE;
struct PID_DATA pid_data;
//Target [ T2 Cycles ] @ 15625 Hz
volatile double target; 
*/

// lcd variables
const int8_t LCD_line1[] PROGMEM = "Current:\0";
const int8_t LCD_line2[] PROGMEM = "Desired:\0";
int8_t lcd_buffer[17];	// LCD display buffer

/********************************************************************/
// 							ISRs & Helper Functions
/********************************************************************/
// --- external interrupt ISR ------------------------
ISR (INT0_vect) {
	

	/*
        motor_period = TCNT2 + motor_period_ovlf  ;
        TCNT2 = 0 ;
        motor_period_ovlf = 0 ;
        flag = TRUE;
    */
}
// --- set up extra 8 bits on timer 2 ----------------
ISR (TIMER2_OVF_vect) {
	/*
        motor_period_ovlf = motor_period_ovlf + 256 ;
        pid_ready = TRUE;
    */
		
}

//**********************************************************
// LCD setup
void init_lcd(void) {
	LCDinit();				//initialize the display
	LCDcursorOFF();
	LCDclr();				//clear the display
	LCDGotoXY(0,0);
}

/*
// **********************************************************
// pwm setup
void init_pwm(){
	DDRB = 0;
	DDRB |= OUTPUT_PIN;
	DDRB |= OUTPUT_PIN2 ;

	TIMSK0 = 0;
	TIMSK2 |= TIMER2_OVERFLOW_INTERRUPT_ENABLE;
	//turn on fast PWM and OC0A - output 
	TCCR0A = 0;
	//divide PWM clock by 1024 
	TCCR0B = 3;// (1 << CS01)  |( 1 << CS00 )  ;
	TCCR0A = (1<<COM0A1) | (1<<COM0B1) | (1 << WGM02) | (1<<WGM00) | (1<<WGM01) ; 

	OCR0A = 128;
	OCR0B = 128;
}
*/

//********************************************************** 
//Set it all up
void initialize(void) {
  DDRC = 0xff;    	// LCD connections
  PORTC = 0x00;
  DDRD |= 0x04;	// LED status light

  //DDRB = 0x00; 		// switch connections
  //PORTB = 0xff; 	// pullup on

  //******************** 
  //initialize variables
  trtWait(SEM_SHARED);
  cTemp = 0.0;
  dTemp = 0.0;
  time_rem = 0;
  trtSignal(SEM_SHARED);

  //******************** 
  //init LCD
  init_lcd();
  LCDclr();
  // put some stuff on LCD
  CopyStringtoLCD(LCD_line1, 0, 0);//start at char=0 line=0
  CopyStringtoLCD(LCD_line2, 0, 1); 


  // ******************** 
  //set up INT0
  //EIMSK = (1 << INT0) ; // turn on int0
  //EICRA = 3 ;       // rising edge
  // turn on timer 2 to be read in int0 ISR
  TCCR2B = 7 ; // divide by 1024
  // turn on timer 2 overflow ISR for double precision time
  TIMSK2 = 1 ;
  
  // ********************
  //set up PWM
  //init_pwm();


  // ********************
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
	volatile int num ;
	char cmd[4] ;
    // initialize
    initialize();
	while (TRUE) {
		// commands:
		// 'temp' sets the desired temperature
		// 'time' sets the desired time
		fprintf(stdout, ">") ;
		fscanf(stdin, "%s%u", cmd, &num) ;
		// update shared variables
		trtWait(SEM_SHARED) ;

		if (strcmp(cmd, "temp") == 0) {
			dTemp = num;
			fprintf(stdout, "dTemp: %i\n\r", dTemp); 
		}
		if (strcmp(cmd, "time") == 0) {
			time_rem = num;
			fprintf(stdout, "Time: %i\n\r", time_rem); 
		}
		if (strcmp(cmd, "set") == 0) {
			cTemp = num;
			fprintf(stdout, "cTemp: %i\n\r", time_rem); 
		}
		/*
		if (cmd[0] == 't' && cmd[1] == 'e') {
			dTemp = num;
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
	  // display the ideal count
	  trtWait(SEM_SHARED) ;
	  sprintf(lcd_buffer, "%iF", cTemp);
	  LCDGotoXY(9, 0);
	  LCDstring(lcd_buffer, strlen(lcd_buffer));
	  // display the actual count 
	  sprintf(lcd_buffer, "%iF", dTemp);
	  LCDGotoXY(9, 1);
	  LCDstring(lcd_buffer, strlen(lcd_buffer));
	  trtSignal(SEM_SHARED) ;
	  // sleep
	  rel = trtCurrentTime() + SECONDS2TICKS(0.2);
	  dead = trtCurrentTime() + SECONDS2TICKS(0.4);
	  trtSleepUntil(rel, dead);	
  	}
}

// --- define task 3  ----------------------------------------
void adjustTemp(void* args) {
	uint32_t rel, dead;
	//double inputValue = 0.0;
	while(TRUE){
		/*
		// detection of the fan speed
		if( flag ) {
			trtWait(SEM_SHARED);
			real_speed = cycles_to_rpm((int) motor_period, 15625);
			flag = FALSE;
			//fprintf(stdout, "RPM: %f\n", real_speed);
			trtSignal(SEM_SHARED);
			pid_ready = FALSE;
			inputValue = pid_Controller(speed, real_speed, &pid_data);
			//fprintf(stdout, "RPM: %f,  delta: %f, target: %f, OCR0A: %d \n\r", real_speed,  inputValue, speed, OCR0A);
			set_input(inputValue);

			//second pwm
			OCR0B = 255 - (real_speed / 12);
		}
		*/
		// sleep
	  	rel = trtCurrentTime() + SECONDS2TICKS(0.02);
	  	dead = trtCurrentTime() + SECONDS2TICKS(0.03);
	  	trtSleepUntil(rel, dead);	
	}
}

// --- Main Program ----------------------------------
int main(void) {
  //init the UART -- trt_uart_init() is in trtUart.c
  trt_uart_init();
  stdout = stdin = stderr = &uart0;
  fprintf(stdout,"\n\r Welcome to KitchenBot UI \n\r Please input your instructions below\n\r\n\r");
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
  trtCreateTask(adjustTemp, 1000, SECONDS2TICKS(0.02), SECONDS2TICKS(0.03), &(args[0]));

  // --- Idle task --------------------------------------
  // For debugging, blink an LED
  // For production, you would probably comment out the body of the WHILE
  while (1) {
  	// Light turn on when it is heating up. 
  	if (cTemp >= dTemp) {
		PORTD |= 0x04;
	}
	else {
		PORTD &= 0xFB;
	}
  /*
  	PORTC = PORTC ^ 0x01 ;
  	_delay_ms(500) ;	
  */
  }

} // main
