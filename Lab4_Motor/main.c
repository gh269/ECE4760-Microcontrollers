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
volatile unsigned int speed;
volatile unsigned int prop_gain;
volatile unsigned int diff_gain;
volatile double int_gain; 

// motor variables
volatile int motor_period;
volatile int motor_period_ovlf;

// lcd variables
const int8_t LCD_initialize[] PROGMEM = "LCD Initialized\0";
const int8_t LCD_line[] PROGMEM = "line 1\0";
const int8_t LCD_number[] PROGMEM = "Number=\0";
int8_t lcd_buffer[17];	// LCD display buffer
uint16_t count;			// a number to display on the LCD  
uint8_t anipos, dir;	// move a character around  

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
    DDRC = 0x01;    	// led connections
    PORTC = 0x00;
    DDRB = 0x00; 		// switch connections
    PORTB = 0xff; 	// pullup on

    //******************** 
    //init LCD
    init_lcd();
    LCDclr;
    // put some stuff on LCD
    CopyStringtoLCD(LCD_line, 8, 1);//start at char=8 line=1	
    CopyStringtoLCD(LCD_number, 0, 0);//start at char=0 line=0
    // init animation state variables	
    count=0;
    anipos = 0;
    LCDGotoXY(anipos,1);	//second line
    LCDsendChar('o');

    //******************** 
	//set up INT0
	EIMSK = 1<<INT0 ; // turn on int0
	EICRA = 3 ;       // rising edge
	// turn on timer 2 to be read in int0 ISR
	TCCR2B = 7 ; // divide by 1024
	// turn on timer 2 overflow ISR for double precision time
	TIMSK2 = 1 ;

	//******************** 
    //init UART0 for PC comm
    UBRR0L = (F_CPU / (16UL * PC_UART_BAUD)) - 1;
    UCSR0B = _BV(TXEN0) ; //| _BV(RXEN1); /* tx/rx enable */
    fprintf(&uart0,"\n\r...Starting IR comm ...\n\r");
  
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
	uint32_t rel, dead;
	// Declare the command and num variables
	double num ;
	char cmd[4] ;
	while (TRUE) {
		// commands:
		// 's' sets the motor speed
		// 'p' sets the proportional gain
		// 'i' sets the differential gain
		// 'd' sets the integral gain
		fprintf(stdout, ">") ;
		fscanf(stdin, "%s%d", cmd, &num) ;
		// update shared variables
		trtWait(SEM_SHARED) ;
		switch (cmd[0]) {
			case 's':	
				speed = (int) num;
				break;
			case 'p':
				prop_gain = (int) num;
				break;
			case 'i':
				diff_gain = (int) num;
				break;
			case 'd': 
				int_gain = num;
				break;
			default:
				break;
		}
		trtSignal(SEM_SHARED);
		// Sleep
	    rel = trtCurrentTime() + SECONDS2TICKS(0.1);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.1);
	    trtSleepUntil(rel, dead);
	}
}

// --- define task 2  ----------------------------------------
void lcdComm(void* args) {
	// increment time counter and format string 
  	sprintf(lcd_buffer,"%-i",count++);	                 
  	LCDGotoXY(7, 0);
  	// display the count 
  	LCDstring(lcd_buffer, strlen(lcd_buffer));	
      	
  	// now move a char left and right
  	LCDGotoXY(anipos,1);	   //second line
  	LCDsendChar(' '); 
      	
  	if (anipos>=7) dir=-1;   // check boundaries
  	if (anipos<=0 ) dir=1;
  	anipos=anipos+dir;
  	LCDGotoXY(anipos,1);	   //second line
  	LCDsendChar('o');
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
  trtCreateTask(serialComm, 100, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[0]));
  trtCreateTask(lcdComm, 100, SECONDS2TICKS(0.2), SECONDS2TICKS(0.3), &(args[0]));

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
