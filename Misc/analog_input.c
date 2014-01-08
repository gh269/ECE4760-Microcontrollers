
#ifndef INPUT_C
#define INPUT_C
//----Analog Input System------


#include "analog_input.h"
//Change thresholds for the different pots to read in analog values
// d - threshold for the 'nose' pot that controls temperature
// d_time - threshold for the minutes and seconds pots
// the time pots are in fact identical pots different from the temperature pot.
int d = 2;
int d_time = 2;

char go_switched(struct ANALOG_INPUT * t){
	return t->current_go_switch;
}
char disp_switched(struct ANALOG_INPUT * t){
	return t->current_disp_switch;
}

// DIAL CHANGED METHODS
char seconds_changed(struct ANALOG_INPUT * t){
	int delta = t->current_seconds - t->prev_seconds;
	return delta > d_time || delta < -d_time;
}

char minutes_changed(struct ANALOG_INPUT * t){
	int delta = t->current_minutes - t->prev_minutes;
	return delta > d_time || delta < -d_time;
}

char temperature_changed(struct ANALOG_INPUT * t){
	int delta = t->current_temp - t->prev_temp;
	return delta > d || delta < -d;
}

long linear_scale(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int pot_to_temp(int value){
	return linear_scale(value, 18, 1000, 0, 100);
}

int pot_to_minutes(int value){
	return linear_scale(value, 0, 1023, 0, 99);
}
int pot_to_seconds(int value){
	return linear_scale(value, 0, 1023, 0, 59);
}


// UTILITY FUNCTIONS
void analog_input_init(struct ANALOG_INPUT * t){

	t->can_tick = 0;
	int pot_sec_reading  =  read_adc(POT_SEC); 
	int pot_min_reading  =  read_adc(POT_MIN); 
	int pot_temp_reading  =  read_adc(POT_TEMP); 

	t->current_disp_switch = (PINB & SWITCH_GO);
	t->current_disp_switch = PINB & SWITCH_DISP;

	t->current_seconds = t->prev_seconds = pot_sec_reading ;
	t->current_minutes = t->prev_minutes = pot_min_reading ;
	t->current_temp    = t->prev_temp    = pot_temp_reading;
}

void analog_input_update(struct ANALOG_INPUT * t){
	
	t->current_go_switch = (PINB & SWITCH_GO);
	t->current_disp_switch = PINB & SWITCH_DISP;
	t->prev_seconds =t->current_seconds;
	t->prev_minutes =t->current_minutes;
	t->prev_temp    =t->current_temp;

	t->current_temp    = read_adc(POT_TEMP); 
	t->current_seconds = read_adc(POT_SEC); 
	t->current_minutes = read_adc(POT_MIN); 
}

#endif 
