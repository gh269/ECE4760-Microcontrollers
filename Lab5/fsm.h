#ifndef FSM_H
#define FSM_H

#include "screen.h"
#include "analog_input.h"

//----------------------------------------------------------------------
// State Definitions
//----------------------------------------------------------------------

#define STATE_HAPPY 0
#define STATE_TEMP_DISPLAY 1
#define STATE_MIN_DISPLAY 2
#define STATE_SEC_DISPLAY 3
#define STATE_CURR_TEMP 4
#define STATE_HOT 5
#define STATE_CURR_TIME 6
#define STATE_DONE 7

//----------------------------------------------------------------------
// State
//----------------------------------------------------------------------

int current_state = STATE_HAPPY;

//----------------------------------------------------------------------
// State Transitions
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// State Outputs
//----------------------------------------------------------------------

void write_state_message_on_buffer(){

	switch (current_state){

		case STATE_HAPPY		: write_happy_to_buffer(); break;
		case STATE_TEMP_DISPLAY : write_temp_to_buffer(pot_to_temp(ant->current_temp)); break;
		case STATE_MIN_DISPLAY  : write_min_to_buffer(pot_to_minutes(ant->current_minutes)); break;
		case STATE_SEC_DISPLAY  : write_sec_to_buffer(pot_to_minutes(antt->current_minutes)); break;

		case STATE_HOT          : write_hot_to_buffer(); break;

		case STATE_DONE         : write_done_to_buffer(); break;

		
		default: write_empty_to_buffer(); break;

	}
}
//
#endif
