
#ifndef INPUT_C
#define INPUT_C
//----Analog Input System------


#include "analog_input.h"
int d = 3;
int d_time = 3;
//BUTTON CHANGE METHODS
/*
char minutes_button_changed(struct ANALOG_INPUT * t){
	int delta = t->current_min_button - t->prev_min_button;
	return delta > 20 || delta < -20;
}

char go_button_changed(struct ANALOG_INPUT * t){
	int delta = t->current_go_button- t->prev_go_button;
	return delta < -20;
}
char temp_button_changed(struct ANALOG_INPUT * t){
	int delta = t->current_temp_button- t->prev_temp_button;
	return delta > 20 || delta < -20;
}
*/


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

/*
char seconds_changed(struct ANALOG_INPUT * t){
	return  t->current_seconds != t->prev_seconds;
}

char minutes_changed(struct ANALOG_INPUT * t){
	return  t->current_minutes != t->prev_minutes;
}


char temperature_changed(struct ANALOG_INPUT * t){
	return t->current_temp != t->prev_temp;
}

char minutes_changed(struct ANALOG_INPUT * t){
	return t->min_dial_fsm->input_state_changed;
}
*/
/*
Performs linear mapping
from original range to new range
*/
long linear_scale(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int pot_to_temp(int value){
	return linear_scale(value, 0, 1023, 0, 100);
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

	//t->temp_dial_fsm = (struct INPUT_DB_FSM * ) malloc(sizeof(struct INPUT_DB_FSM));
	//t->min_dial_fsm = (struct INPUT_DB_FSM * ) malloc(sizeof(struct INPUT_DB_FSM));
	//t->sec_dial_fsm = (struct INPUT_DB_FSM * ) malloc(sizeof(struct INPUT_DB_FSM));

	int pot_sec_reading =  read_adc(POT_SEC); 
	int pot_min_reading =  read_adc(POT_MIN); 
	int pot_temp_reading =  read_adc(POT_TEMP); 

    //create_input_db_fsm_test(t->temp_dial_fsm, pot_sec_reading  );
	//create_input_db_fsm_test(t->min_dial_fsm , pot_min_reading  );
	//create_input_db_fsm_test(t->sec_dial_fsm , pot_temp_reading );
	/*
	t->current_min_button = t->prev_min_button = read_adc(BUTT_MIN);
	t->current_go_button  = t->prev_go_button  = read_adc(BUTT_GO);
	t->current_temp_button= t->current_temp_button = read_adc(BUTT_TEMP);
	*/
	t->current_seconds = t->prev_seconds = pot_sec_reading ;
	t->current_minutes = t->prev_minutes = pot_min_reading ;
	t->current_temp    = t->prev_temp    = pot_temp_reading;
}

void analog_input_update(struct ANALOG_INPUT * t){
	/*
	if( t -> can_tick == 1){
		tick_fsm(t->temp_dial_fsm);
		tick_fsm(t->min_dial_fsm );
		tick_fsm(t->sec_dial_fsm );
		t->can_tick = 0;
	}
	*/
	/*
	t->prev_min_button = t->current_min_button;
	t->current_min_button = read_adc(BUTT_MIN);

	t->prev_go_button  = t->current_go_button;
	t->current_go_button  = read_adc(BUTT_GO);

	t->prev_temp_button = t->current_temp_button;
	t->current_temp_button = read_adc(BUTT_TEMP);
	*/
	t->current_go_switch = (PINB & SWITCH_GO);
	t->current_disp_switch = PINB & SWITCH_DISP;
	//int pot_sec_reading =  read_adc(POT_SEC); 
	//int pot_min_reading =  read_adc(POT_MIN); 
	//int pot_temp_reading = read_adc(POT_TEMP); 

	//update_input(t->temp_dial_fsm, pot_temp_reading );
	//update_input(t->min_dial_fsm , pot_min_reading );
	//update_input(t->sec_dial_fsm , pot_sec_reading);

	t->prev_seconds =t->current_seconds;
	t->prev_minutes =t->current_minutes;
	t->prev_temp    =t->current_temp;

	t->current_temp    = read_adc(POT_TEMP); 
	t->current_seconds = read_adc(POT_SEC); 
	t->current_minutes = read_adc(POT_MIN); 
	/*
	t->current_temp    = t->temp_dial_fsm->input_state;
	t->current_seconds = t->sec_dial_fsm ->input_state;
	t->current_minutes = t->min_dial_fsm ->input_state;
	*/
}

#endif 
