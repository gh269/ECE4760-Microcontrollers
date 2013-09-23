                 
// Keyscanner
// Mega644 version 
// assumes a standard 4x4 keypad connected to a port (example is PORTD)
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/eeprom.h>
      
#include "uart.h"

#define maxkeys 16
#define PORTDIR DDRA
#define PORTDATA PORTA
#define PORTIN PINA

#define begin {
#define end }

// The raw keyscan
unsigned char key ;   
// The decoded button number
unsigned char butnum ;

//key pad scan table
/*unsigned char keytbl[16]={0xee, 0xed, 0xeb, 0xe7, 
						  0xde, 0xdd, 0xdb, 0xd7, 
						  0xbe, 0xbd, 0xbb, 0xb7, 
						  0x7e, 0x7d, 0x7b, 0x77};
*/
unsigned char keytbl[16] = {0x77, 0x7b, 0x7d, 0x7e,
							0xb7, 0xbb, 0xbd, 0xbe,
							0xd7, 0xdb, 0xdd, 0xde,
							0xe7, 0xeb, 0xed, 0xee};


// UART file descriptor
// putchar and getchar are in uart.c
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

int main(void)
begin
	//DDRD=0x02;	// PORT D is an input except for UART on D.1
  // Init port B to show keyboard result
  DDRB = 0xff;
  // and turn out the LEDs
  PORTB = 0xff;   
  
  // init the UART
  uart_init();
  stdout = stdin = stderr = &uart_str;
  fprintf(stdout, "Starting...\n\r");
  
  	// find out if eeprom has EVER been written
  /*
  if (eeprom_read_byte((uint8_t*)eeprom_true) != 'T')
  begin // if not, write some data and the "written" flag
  	eeprom_write_word((uint16_t*)eeprom_data,0);
	eeprom_write_byte((uint8_t*)eeprom_true,'T');
  end   
	*/
  //endless loop to read keyboard
  while(1)
  begin 
	//get lower nibble
  	PORTDIR = 0x0f;
  	PORTDATA = 0xf0; 
  	_delay_us(5);
  	key = PORTIN;
  	  
  	//get upper nibble
  	PORTDIR = 0xf0;
  	PORTDATA = 0x0f; 
  	_delay_us(5);
  	key = key | PORTIN;
  	  
  	//find matching keycode in keytbl
  	if (key != 0xff)
  	begin   
  	  for (butnum=0; butnum<maxkeys; butnum++)
  	  begin   
  	  	if (keytbl[butnum]==key)  break;   
  	  end

  	  if (butnum==maxkeys) butnum=0;
  	  else butnum++;	   //adjust by one to make range 1-16
  	end  
  	else butnum=0;

    // init the DDS phase increment
    // for a 32-bit DDS accumulator, running at 16e6/256 Hz:
    // increment = 2^32*256*Fout/16e6 = 68719 * Fout
    // Fout=1000 Hz, increment= 68719000 
    increment = 68719000L ;

	switch (butnum) {
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		case 4: 
			break;
		case 5:
			break;
		case 6:
			break;
		case 7:
			break;
		case 8:
			break;
		case 9: 
			break;
		case 10: 
			break;
		case 11:
			break;
		case 12:
			break;
		case 13:
			break;
		case 14:
			break;
		case 15:
			break;
		case 16:
			break;
		default:
			break;
	}

  	if (butnum != 0){ 
		fprintf(stdout, "%d\n\r",butnum);
  	}
	//PORTB = ~butnum ;
  	
  	end // end while
  end   //end main
