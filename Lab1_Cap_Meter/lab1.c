/*
Tasks to complete:

The LCD should be updated every 200 mSec or so. (DONE)
An LED should blink about 1/second. (DONE)
The capacitance should be measured as quickly as possible as described above.

	Your program will have to (in time order):

	Set PortB3 to an input.
	Drive PortB2 to zero by making it an output and wait long enough to discharge the capacitor through 100 ohms. Clearly, to dischage to zero volts with 1% accuracy, R2>100*(100ohms).
	Convert PortB2 to an input and start a timer. The capacitor will start to charge toward Vcc.
	Detect when the voltage at PortB2 is greater than than the voltage at PortB3. That is, you will have to record when the comparator changes state. You could do this by polling the ACO bit of the ACSR and stopping the clock when ACO changes state, but a much better way to do it is to use the timer1 input capture function set up to be triggered by the comparator. Using input capture gives better timing accuracy and more dynamic range.
	Repeat


The range of capacitances to be measured is 1 nf to 100 nf.
The program should detect whether a capacitance is present or not and display an appropriate message if no capacitor is present.
If present, format the capacitance as an ASCII number and prints the message C = xxx nf to the LCD
*/
#define F_CPU 16000000UL                
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h> // needed for lcd_lib
#include "lcd_lib.h"

//---------------Capacitance Measurements---------------
//capacitance measurement bits
//Analog Comparator negative ( - ) input - reference Port B3
#define COMPARATOR_REFERENCE 0x08
//Capacitor input voltage ( + ) input - port B2
#define COMPARATOR_INPUT 0x04

//Discharge Period [ units: us ]
#define CAP_DISCHARGE_PERIOD 45
// Each of each count (16Mhz) [ units: ns]
#define T1_CLK_PERIOD 62.5
// The R2 resistor value [ units: Ohms]
#define RESISTOR 10000

//----------timer register variables----------------------
#define INPUT_CAPTURE_EDGE_SELECT (1 << ICES1)
#define INTERRUPT_ON_CAPTURE (1 << ICIE1)
#define INTERRUPT_ON_OVERFLOW (1 << TOIE1)

//analog comp register variables
#define ANALOG_COMPARATOR_INPUT_CAPTURE_ENABLE (1 << ACIC)
#define ANALOG_COMPARATOR_BANDGAP_SELECT (1 << ACBG )
#define INTERRUPT_OVERFLOW (1<<TOIE1)
//---------------LED Bits---------------------
#define ONBOARD_LED 0x04 //LED is on D.2
// period of LED blinking  [ units : ms ]
#define LED_BLINK_PERIOD 1000

#define TRUE 1
#define FALSE 0

//---------------Timer TCCR0B Bits-------------
#define T0B_CS02 4
#define T0B_CS01 2
#define T0B_CS00 1
#define CLEAR_ON_MATCH (1 << WGM01)
#define OUTPUT_COMPARE_A0_MATCH_INTERRUPT_ENABLE (1 << OCIE0A)
#define OUTPUT_COMPARE_A1_MATCH_INTERRUPT_ENABLE (1 << OCIE1A)

//---------------LCD variables------------------
#define LCD_REFRESH_RATE 200

const uint8_t LCD_initialize[] PROGMEM = "LCD Initialized\0";
const uint8_t LCD_number[] PROGMEM = "Capacitance=\0";
uint8_t lcd_buffer[17];	// LCD display buffer
uint16_t count;			// a number to display on the LCD  
uint8_t anipos, dir;	// move a character around  

// Flags for the finite state machine transitions
volatile unsigned int begin_cap_measurement;
volatile unsigned int cap_discharged;
volatile unsigned int cap_charged;

//time counter for LED blinking
volatile unsigned int led_time_count;
//time counter for LCD refresh
volatile unsigned int lcd_time_count;

// timer 1 capture variables for computing charging time
volatile int charge_cycles;	
volatile double charge_time; 
// variable to store capacitance for print out
volatile double capacitance;
// precomputed log(.5) needed for capacitance calculation
const double ln_half = 0.6931471805599453;

volatile int overflow = FALSE;

//configures Analog Comparator and Timer1
//set it to full speed 
//clear TCNT1
void init_cap_measurement_analog_timer(){
	TCCR1B = 0;
	//full speed [ 16 MHz], capture on positive edge
	TCCR1B |= INPUT_CAPTURE_EDGE_SELECT + T0B_CS00;
	//turn on timer 1 interrupt-on-capture
	TIMSK1 = 0;
	TIMSK1 |= INTERRUPT_ON_CAPTURE + INTERRUPT_OVERFLOW;

	//set analog comp to connect to timer capture input
	//with positive input reference voltage
	ACSR = 0;
	ACSR |= ANALOG_COMPARATOR_INPUT_CAPTURE_ENABLE;
	ACSR &= ~ANALOG_COMPARATOR_BANDGAP_SELECT;
	//set all ports to input
	DDRB = 0;
	DDRB &= ~(COMPARATOR_INPUT + COMPARATOR_REFERENCE);
}
//Overflow ISR
ISR(TIMER1_OVF_vect)
{
	//increment overflow counter
	overflow = TRUE;
}
//Uses Timer1.A to wait 
//sets Timer1.A into a 1 MHz frequency 
void init_cap_discharge_wait_timer(){
	// Output capture/compare on OCR1A IE
	TIMSK1 = OUTPUT_COMPARE_A1_MATCH_INTERRUPT_ENABLE;
	OCR1A = 2 * CAP_DISCHARGE_PERIOD;
	//CS1 sets prescaler to div by 8 - clock 
	// 16 MHz				2 MHz
	// -------  = 2 MHz;  ------------------   = CAP_DISCHARGE period
	//    8                2 * CAP_DISCHARGE
	TCCR1B = T0B_CS01;
	//turn on clear on match
	TCCR1A = 0;
	TCCR1A |= CLEAR_ON_MATCH;
}

/*
	Set PortB3 to an input.
	Drive PortB2 to zero by making it an output and wait long enough to discharge the capacitor through 100 ohms. Clearly, to dischage to zero volts with 1% accuracy, R2>100*(100ohms).
	Convert PortB2 to an input and start a timer. The capacitor will start to charge toward Vcc.
	Detect when the voltage at PortB2 is greater than than the voltage at PortB3. That is, you will have to record when the comparator changes state. You could do this by polling the ACO bit of the ACSR and stopping the clock when ACO changes state, but a much better way to do it is to use the timer1 input capture function set up to be triggered by the comparator. Using input capture gives better timing accuracy and more dynamic range.
	Repeat
*/
void init_cap_measurements(void){
	//Reset all measurements
	//capacitance = 0;
	//charge_cycles = 0;
	//charge_time = 0;

	DDRB = 0;
	//set B3 to an input
	//make the reference an input to the Analog Comparator
	DDRB &= ~COMPARATOR_REFERENCE;
	//Drive B2 to 0 by making it an output and waiting long enough to discharge the cap
	DDRB |= COMPARATOR_INPUT;
	PORTB &= ~COMPARATOR_INPUT;
	//use Timer1.A to perform this delay and signal when we can continue measurements
	init_cap_discharge_wait_timer();
}

//1 ms timebase register
//Blinks LED 1/second
//refreshes LCD 1/200 ms
ISR (TIMER0_COMPA_vect){
	if( led_time_count > 0)
		--led_time_count;
	if( lcd_time_count > 0)
		--lcd_time_count;

}
//Once this triggers even once, we know that we have waited long enough for a cap discharge
ISR (TIMER1_COMPA_vect){
	cap_discharged = TRUE;
}

// timer 1 capture ISR to measure charging time. 
// and the counter should have been initialized at zero. 
// when this Interrupt triggers, the voltage of the cap 
// is at VCC/2 
/*
	Charging Equation:

	Vcap = Vcc ( 1 - e ^ (-t /tau ))
	0.5 = 1 - e ^ (-t / tau)
	e^(-t / tau ) = 0.5
	- (t / tau ) = ln(0.5)
			   t
	tau = ---------- = R2 * C
		   ln(0.5) 
			
			  t
	C = ------------
		R2 * ln(0.5)

*/
ISR (TIMER1_CAPT_vect){
	// read timer1 input capture register
    charge_cycles = ICR1;
	ICR1 = 0;
    // set the charged flag to true
    cap_charged = TRUE;
}

//
//Blinks the ONBOARD_LED D.2
void toggle_led(void){
	PORTD ^= ONBOARD_LED;
}

//setup timer 0 for a 1 ms timebase
// triggers the ISR on TIMER0_COMPA_vect
// on TCNT0 = OCR0A
void init_timer0A(void){
	// Output capture/compare on OCR0A IE
	TIMSK0 = OUTPUT_COMPARE_A0_MATCH_INTERRUPT_ENABLE;
	OCR0A = 249;
	//T0BCS01 + T0BCS00 sets clk divider 64
	// 16 MHz 				250 KHz
	// ------  = 256 KHz;  ---------  = 1 KHz  = 1ms period
	//   64					OCR0A=249
	TCCR0B = T0B_CS01 + T0B_CS00;
	//turn on clear-on-match - timer A ISR will clear TCNT0 on match
	TCCR0A = 0;
	TCCR0A |= CLEAR_ON_MATCH;
}

// LCD setup
void init_lcd(void){
	LCDinit();	//initialize the display
	LCDcursorOFF();
	LCDclr();				//clear the display
	LCDGotoXY(0,0);
	CopyStringtoLCD(LCD_number, 0, 0);
}


// writes contents of lcd_buffer to LCD every 200 mSec
// 
void refresh_lcd(void){
  // increment time counter and format string 
  //if (capacitance >= .1 && capacitance <= 100) {
  //if (charge_cycles > 200) {
  //sprintf(lcd_buffer,"%-.5f",capacitance);
  sprintf(lcd_buffer,"%-i", charge_cycles);	 
  //}
  //else {
  //	sprintf(lcd_buffer,"N/A");
  //}               
  LCDGotoXY(0, 1);
  	// display the capacitance 
  LCDstring(lcd_buffer, strlen(lcd_buffer));	
  // now move a char left and right
  LCDGotoXY(anipos,1);	   //second line
  LCDsendChar(' '); 
      	
  if (anipos>=15) dir=-1;   // check boundaries
  if (anipos<=8 ) dir=1;
  anipos=anipos+dir;
  LCDGotoXY(anipos,1);	   //second line
  LCDsendChar('o');
}

void initialize(void){
	anipos = 8;
	led_time_count = 0;
	init_timer0A();

	DDRB = 0;
	DDRD = 0;

	//Enable LED Port
	DDRD = ONBOARD_LED; //turn the LED to an output
	PORTD = 0xFF; //turn off LED 

	capacitance = 0;
	charge_cycles = 0;
	charge_time = 0;
	cap_discharged = FALSE;
	begin_cap_measurement = FALSE;
	cap_charged = FALSE;

	init_lcd();
	LCDclr();


	sei();
}

int main(void){
	initialize();
	CopyStringtoLCD(LCD_number, 0, 0);//start at char=0 line=0
	
	while(1){
		if( led_time_count == 0){
			led_time_count = LED_BLINK_PERIOD / 2;
			toggle_led();
		}
		if( lcd_time_count == 0){
			lcd_time_count = LCD_REFRESH_RATE;
			refresh_lcd();
		}
		if (!cap_discharged && !begin_cap_measurement && !cap_charged) {
			charge_cycles = 0;
			init_cap_measurements();
		}
		if(cap_discharged && !begin_cap_measurement){
			//begin cap measurements
			//switch Timer1A mode
			//DDRB &= ~COMPARATOR_INPUT;
			//mark that we can start cap measurement
			begin_cap_measurement = TRUE;
			//initalize timer for cap measurement
			init_cap_measurement_analog_timer();
		}
		if(overflow){
			sprintf(lcd_buffer, "OVERFLOW\0");
			LCDGotoXY(0,0);
  			LCDstring(lcd_buffer, strlen(lcd_buffer));	


		}
		if(begin_cap_measurement && cap_charged){
			// Revert the flags
			cap_discharged = FALSE;
			begin_cap_measurement = FALSE;
			cap_charged = FALSE;
			// Calculate the capacitance with the time elapsed. 
			// V(t) = Vo(1 - exp(-t/(R2*C))) becomes
			// C = -t / (R2 * ln(.5)) to find out when V(t) = .5 * Vo (R3 = R4)
			// (Due to ln(.5) being negative, the negative on the t is canceled out)
			//charge_time = charge_cycles * T1_CLK_PERIOD;
			//capacitance = charge_time / (RESISTOR * ln_half);
			
			//capacitance = charge_cycles / (RESISTOR * ln_half);
			//capacitance = capacitance / T1_CLK_PERIOD;
		}
	}
}

