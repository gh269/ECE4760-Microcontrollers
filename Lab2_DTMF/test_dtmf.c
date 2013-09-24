// Keyscanner
// Mega644 version 
// assumes a standard 4x4 keypad connected to a port
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
      
#include "uart.h"
#include "dds.h"
#include "atmega1284p.h"


int main(){
	init_dtmf();
	while(1){
		play(1209,0,1000);
		if(time >= dds_duration){
			stop_playing();
		}
	}
}