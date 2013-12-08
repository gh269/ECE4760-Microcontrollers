#ifndef INPUT_DB_FSM_H
#define INPUT_DB_FSM_H

#define STATE_READING 0
#define STATE_DB_DELAY 1
#define STATE_SET_NEW 2
// binary states
#define FALSE 0
#define TRUE 1

//debounce delay in millis
#define DEBOUNCE_DELAY 3

typedef struct INPUT_DB_FSM{
	int state;

	int prev_input_state;

	int input_state;
  char debouncing;
  int debounce_time;

} input_db_t;

void create_input_db_fsm(struct INPUT_DB_FSM * fsm ){
  fsm -> debouncing = FALSE;
  fsm -> debounce_time = 0;
  
  fsm -> state = STATE_READING;
  
  fsm -> input_state = 0;
  fsm -> prev_input_state = 0;
}

void tick_fsm(struct INPUT_DB_FSM *fsm){
  if(fsm->debouncing && fsm -> debounce_time > 0){
    fsm->debounce_time--;
  }
}

void update_input(struct INPUT_DB_FSM * fsm , int pin){
  int reading = pin;
  int next_state;
  switch(fsm -> state){
    case STATE_READING: if( reading != fsm->prev_input_state){ 
                          next_state = STATE_DB_DELAY; 
                           fsm->debouncing = FALSE; 
                         }
                         break;
    case STATE_DB_DELAY: if(fsm->debouncing && !fsm -> debounce_time) next_state = STATE_SET_NEW;
                         else if(fsm -> debouncing && fsm -> debounce_time) next_state = STATE_DB_DELAY;
                         else{
                           fsm -> debouncing = TRUE;
                           fsm -> debounce_time = DEBOUNCE_DELAY;
                           next_state = STATE_DB_DELAY;
                         }
                         break;
    case STATE_SET_NEW: fsm -> input_state = reading; next_state = STATE_READING;
                        break;
    default:            next_state = STATE_READING;
                        break;              
  }
  fsm -> prev_input_state = reading;
  fsm -> state = next_state;
}


#endif 