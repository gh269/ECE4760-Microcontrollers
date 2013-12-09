#ifndef FSM_H
#define FSM_H

#include "screen.h"
#include "analog_input.h"
#include "trtkernel_1284.c"
#include "regulate.h"
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
#define STATE_GO 8
#define STATE_TARGET_TIME 9 
#define STATE_BEEP_ONCE 10
#define STATE_TIME_REM 11
#define STATE_CURR_TEMP_COOK 12
#define STATE_DONE_BEEP 13
//----------------------------------------------------------------------
// State
//----------------------------------------------------------------------

volatile int current_state = STATE_HAPPY;
volatile int next_state    = STATE_HAPPY;


int convert_time_to_seconds(int minutes, int seconds){
	return 60 * minutes + seconds;
}


//----------------------------------------------------------------------
// State Outputs
//----------------------------------------------------------------------

void write_state_message_on_buffer(){

	switch (current_state){

		case STATE_HAPPY		: write_happy_to_buffer(); break;
		//Display States - no associated logic
		case STATE_TEMP_DISPLAY : write_temp_to_buffer(pot_to_temp(ant->current_temp)); break;
		case STATE_MIN_DISPLAY  : write_min_to_buffer(pot_to_minutes(ant->current_minutes)); break;
		case STATE_SEC_DISPLAY  : write_sec_to_buffer(pot_to_seconds(ant->current_seconds)); break;

		case STATE_CURR_TEMP    : write_temp_to_buffer(dTemp); break;
		case STATE_TARGET_TIME  : write_time_to_buffer(time_rem);  break;


		case STATE_BEEP_ONCE    : write_done_to_buffer(); break;

		case STATE_TIME_REM     : trtWait(SEM_SHARED) ; write_time_to_buffer(time_rem);  trtSignal(SEM_SHARED); break;
		case STATE_DONE         : write_done_to_buffer(); break;
		case STATE_GO           : write_empty_to_buffer(); break;
		default					: write_empty_to_buffer(); break;

	}
}

void handle_next_state_logic(){
	switch(next_state){

		case STATE_GO 			: trtWait(SEM_SHARED) ;
								  dTemp = pot_to_temp(ant->current_temp); 
								  time_rem = convert_time_to_seconds(pot_to_minutes(ant->current_minutes),
								  									 pot_to_seconds(ant->current_seconds));
								 trtSignal(SEM_SHARED); 
								 break;

		case STATE_BEEP_ONCE	: beep_timer = BEEP_ONCE_TIME;
								  PORTD |= SOUND_EN; 
								  while(beep_timer > 0); 
								  PORTD &= ~SOUND_EN; 
								  count_en = 1; 
								  break; 

		case STATE_DONE_BEEP    : PORTD |= SOUND_EN;
								  while(go_switched(ant)){
								  	analog_input_update(ant);
								  } 
								  PORTD &= ~SOUND_EN; 
								  break;

		default					: break;
	}
}

//
#endif
