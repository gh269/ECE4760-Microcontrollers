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
	byte current_go_button;
	byte prev_go_button;

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

char minutes_button_changed(struct ANALOG_INPUT * t);
char go_button_changed(struct ANALOG_INPUT * t);
char temp_button_changed(struct ANALOG_INPUT * t);

char seconds_changed(struct ANALOG_INPUT * t);
char minutes_changed(struct ANALOG_INPUT * t);
char temperature_changed(struct ANALOG_INPUT * t);



#endif