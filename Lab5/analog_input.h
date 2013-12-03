#ifndef INPUT_H
#define INPUT_H

/*
	MIN	  SEC
 	  TEMP
*/
typedef struct ANALOG_INPUT{
	//-----BUTTONS-----------
	//min is set time
	byte current_min_button;
	byte prev_min_button;

	//sec is GO
	byte current_sec_button;
	byte prev_sec_button;

	//temp is set temp
	byte current_temp_button;
	byte prev_temp_button;
	//-----------------------

	//------DIALS------------
	int current_seconds;
	int prev_seconds;

	int current_minutes;
	int prev_minutes;

	int current_temp;
	int prev_temp;
	//-----------------------
	
} analog_input_t;


char seconds_changed();
char minutes_changed();
char temperature_changed();

#endif