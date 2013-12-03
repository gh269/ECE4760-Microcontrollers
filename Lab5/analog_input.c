//----Analog Input System------

#include "analog_input.h"


char seconds_changed(){
	return (current_seconds != prev_seconds);
}

char minutes_changed(){
	return (current_minutes != prev_minutes);
}

char temperature_changed(){
	return (current_temperature != prev_temperature);
}


void 