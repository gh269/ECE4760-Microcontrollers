
#ifndef INPUT_C
#define INPUT_C
//----Analog Input System------

#include "analog_input.h"

char minutes_button_changed(struct ANALOG_INPUT * t);
char go_button_changed(struct ANALOG_INPUT * t);
char temp_button_changed(struct ANALOG_INPUT * t);

char seconds_changed(struct ANALOG_INPUT * t);
char minutes_changed(struct ANALOG_INPUT * t);
char temperature_changed(struct ANALOG_INPUT * t);

void analog_input_init(struct ANALOG_INPUT * t);
void analog_input_update(struct ANALOG_INPUT * t);




#endif 
