#ifndef INPUT_H
#define INPUT_H

#define POT_MIN 1
#define POT_SEC 2
#define POT_TEMP 3

#define BUTT_MIN 4
#define BUTT_GO 5
#define BUTT_TEMP 6

#define BUTTON_MIN 0x80
#include "input_db.h"
/*
	MIN	  SEC
 	  TEMP
*/
typedef struct ANALOG_INPUT{
	//-----BUTTONS-----------
	//min is set time
	char current_min_button;
	char prev_min_button;

	//sec is GO
	char current_go_button;
	char prev_go_button;

	//temp is set temp
	char current_temp_button;
	char prev_temp_button;
	//-----------------------

	//------DIALS------------
	int current_seconds;
	int prev_seconds;

	int current_minutes;
	int prev_minutes;

	int current_temp;
	int prev_temp;
	//-----------------------

	//-----FSMS--------------
	struct INPUT_DB_FSM * temp_dial_fsm;
	struct INPUT_DB_FSM * go_fsm;
	struct INPUT_DB_FSM * sec_dial_fsm;
	int can_tick;
} analog_input_t;

 struct ANALOG_INPUT * ant;


uint16_t read_adc(uint8_t channel);
char minutes_button_changed(struct ANALOG_INPUT * t);
char go_button_changed(struct ANALOG_INPUT * t);
char temp_button_changed(struct ANALOG_INPUT * t);
long linear_scale(long x, long in_min, long in_max, long out_min, long out_max);

int pot_to_temp(int value);
int pot_to_minutes(int value);
int pot_to_seconds(int value);

char seconds_changed(struct ANALOG_INPUT * t);
char minutes_changed(struct ANALOG_INPUT * t);
char temperature_changed(struct ANALOG_INPUT * t);

void analog_input_init(struct ANALOG_INPUT * t);
void analog_input_update(struct ANALOG_INPUT * t);


#endif
