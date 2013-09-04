/**
 * A Mega644 LCD Test
 *
 * Written by Ruibing Wang (rw98@cornell.edu)
 * Mods for 644 by brl4@cornell.edu
 * Feb 2010
 */

/** 
 * Use an 2x16 alphanumeric LCD connected to PORTC as follows:
 *  
 *	[LCD] -	[Mega644 Pin]
 *	1 GND -	GND
 *	2 +5V -	VCC
 *	3 VLC 10k trimpot wiper (trimpot ends go to +5 and gnd) 
 *	4 RS -	PC0
 *	5 RD -	PC1
 *	6 EN -	PC2
 *
 *	11 D4 -	PC4
 *	12 D5 -	PC5
 *	13 D6 -	PC6
 *	14 D7 -	PC7 
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h> // needed for lcd_lib
#include "lcd_lib.h"

const int8_t LCD_initialize[] PROGMEM = "LCD Initialized\0";
const int8_t LCD_line[] PROGMEM = "line 1\0";
const int8_t LCD_number[] PROGMEM = "Number=\0";

int8_t lcd_buffer[17];	// LCD display buffer
uint16_t count;			// a number to display on the LCD  
uint8_t anipos, dir;	// move a character around  

//timeout value for LCD task
#define t1 200 
// running time counter for task
volatile unsigned char time1 ;

// redefine brackets
#define begin {
#define end }

//**********************************************************
// LCD setup
void init_lcd(void) 
begin
	LCDinit();	//initialize the display
	LCDcursorOFF();
	LCDclr();				//clear the display
	LCDGotoXY(0,0);
	CopyStringtoLCD(LCD_initialize, 0, 0);
end	

//**********************************************************
//timer 0 compare ISR
ISR (TIMER0_COMPA_vect)
begin      
  //Update the task time
  if (time1>0)	--time1;
end  

//**********************************************************
// task 1 writes to LCD every 200 mSec
void task1 (void) 
begin
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
end

//**********************************************************       
//Entry point and task scheduler loop
int main(void)
begin  
  
  // start the LCD 
  init_lcd();
  LCDclr();

  //init the task timer
  time1=t1;
  //set up timer 0 for 1 mSec ticks
  TIMSK0 = 2;		//turn on timer 0 cmp match ISR 
  OCR0A = 249;  	//set the compare reg to 250 time ticks
  TCCR0A = 0b00000010; // turn on clear-on-match
  TCCR0B = 0b00000011;	// clock prescalar to 64
  
  // put some stuff on LCD
  CopyStringtoLCD(LCD_line, 8, 1);//start at char=8 line=1	
  CopyStringtoLCD(LCD_number, 0, 0);//start at char=0 line=0
  // init animation state variables	
  count=0;
  anipos = 0;
  LCDGotoXY(anipos,1);	//second line
  LCDsendChar('o');
  
  sei();
  //main task scheduler loop 
  while(1)
  begin 
  	// reset time and call task    
    if (time1==0){ time1=t1; task1();}
  end
end 

//**********************************************************  



