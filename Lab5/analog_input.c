
#ifndef INPUT_C
#define INPUT_C
//----Analog Input System------


#include "analog_input.h"

//BUTTON CHANGE METHODS
char minutes_button_changed(struct ANALOG_INPUT * t){
	int delta = t->current_min_button - t->prev_min_button;
	return delta > 20 || delta < -20;
}

char go_button_changed(struct ANALOG_INPUT * t){
	int delta = t->current_go_button- t->prev_go_button;
	return delta > 20 || delta < -20;
}
char temp_button_changed(struct ANALOG_INPUT * t){
	int delta = t->current_temp_button- t->prev_temp_button;
	return delta > 20 || delta < -20;
}
// DIAL CHANGED METHODS
char seconds_changed(struct ANALOG_INPUT * t){
	int delta = t->current_seconds - t->prev_seconds;
	return delta > 2 || delta < -2;
}

char minutes_changed(struct ANALOG_INPUT * t){
	int delta = t->current_seconds - t->prev_seconds;
	return delta > 2 || delta < -2;
}

char temperature_changed(struct ANALOG_INPUT * t){
	int delta = t->current_temp - t->prev_temp;
	return delta > 2 || delta < -2;
}


// UTILITY FUNCTIONS
void analog_input_init(struct ANALOG_INPUT * t){
	t->current_min_button = t->prev_min_button = read_adc(BUTT_MIN);
	t->current_go_button  = t->prev_go_button  = read_adc(BUTT_GO);
	t->current_temp_button= t->current_temp_button = read_adc(BUTT_TEMP);

	t->current_seconds = t->prev_seconds = read_adc(POT_SEC);
	t->current_minutes = t->prev_minutes = read_adc(POT_MIN);
	t->current_temp    = t->prev_temp    = read_adc(POT_TEMP);
}

void analog_input_update(struct ANALOG_INPUT * t){
	t->prev_min_button = t->current_min_button;
	t->current_min_button = read_adc(BUTT_MIN);

	t->prev_go_button  = t->current_go_button;
	t->current_go_button  = read_adc(BUTT_GO);

	t->prev_temp_button = t->current_temp_button;
	t->current_temp_button = read_adc(BUTT_TEMP);

	t->prev_seconds = t->current_seconds;
	t->current_seconds = read_adc(POT_SEC);

	t->prev_minutes = t->current_minutes;
	t->current_minutes = read_adc(POT_MIN);

	t->prev_temp = t->current_temp;
	t->current_temp = read_adc(POT_TEMP);
}

#endif 
