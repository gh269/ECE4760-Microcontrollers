#ifndef SCREEN_C
#define SCREEN_C

#include "screen.h"

void write_buffers_to_screen(){
	int i, j, temp_left, temp_right;
	int val_left = 0;
	int val_right = 0;

	for( i = 1; i < 7; i++){
		PORTC |= row_order[i];

		val_left = displaybuffer_left[i];
		val_right = displaybuffer_right[i];
		for( j = 1; j <=8; j++){
			temp_left = val_left & 0x01;
			temp_right = val_right & 0x01;

			if(temp_left) 
				PORTB |= SCREEN_LEFT;
			else
				PORTB &= ~SCREEN_LEFT;

			if(temp_right)
				PORTB |= SCREEN_RIGHT;
			else
				PORTB &= ~SCREEN_RIGHT;

			PORTB |= CLK;
			_delay_us(1);
			PORTB &= ~CLK;
			
			val_left = val_left >> 1;
			val_right = val_right >> 1;
		}
		_delay_us(ROW_SCAN_DELAY);
		//delay_row_scan(ROW_SCAN_DELAY);
		PORTC &= ~row_order[i];
	}
}

//Write to whole screen
void write_to_buffer(int * left_valueR, int * left_valueL, int * right_valueR, int * right_valueL){
	int i;
	for( i = 0; i <8; i++){
		displaybuffer_left[i] = left_valueR[i] | (left_valueL[i] << 4);
		displaybuffer_right[i] = right_valueR[i] | (right_valueL[i] << 4);
	}
}


void write_two_values_to_buffer(int * valueR, int * valueL, int screen){
	int i;
	for( i = 0; i < 8; i++){
		if(screen == SCREEN_LEFT){
			displaybuffer_left[i] = valueR[i] | (valueL[i] << 4);
		}
		else{
			displaybuffer_right[i] = valueR[i] | (valueL[i] << 4);
		}
	}
}

void write_happy_to_buffer(){
	write_to_buffer(mouth[MOUTH_LEFT], space, space, mouth[MOUTH_RIGHT] );
}

void write_hot_to_buffer(){
	//                Left.R      Left.L       Right.R        Right.L
	write_to_buffer(numbers[0], hot[H_IDX], hot[EXCL_IDX], hot[T_IDX]);
}

void write_empty_to_buffer(){
	write_to_buffer(space, space, space, space);
}

void write_temp_to_buffer(int temp_celsius){
	//int temp_celsius = pot_to_temp(t->current_temp);
	int * left_valueL = ( temp_celsius < 100 ) ? numbers[0] : numbers[1];
	int * left_valueR = numbers[(temp_celsius / 10 )% 10];
	int * right_valueL = numbers[temp_celsius % 10];
	write_to_buffer(left_valueR, left_valueL, deg_celsius, right_valueL);
}

void write_min_to_buffer(int min){
	//int min = pot_to_minutes(t->current_minutes);
	int * right_valueR = space;
	int * right_valueL = minutes;
	int * left_valueR = (min < 1) ? numbers[0] : numbers[min % 10];
	int * left_valueL = numbers[min / 10];
	write_to_buffer(left_valueR, left_valueL, right_valueR, right_valueL);
}

void write_sec_to_buffer(int sec){
	//int sec = pot_to_minutes(t->current_minutes);
	int * right_valueR = space;
	int * right_valueL = seconds;
	int * left_valueR = (sec < 1) ? numbers[0] : numbers[sec % 10];
	int * left_valueL = numbers[sec / 10];
	write_to_buffer(left_valueR, left_valueL, right_valueR, right_valueL);
}

void write_done_to_buffer(){
	write_to_buffer(done[MOUTH_LEFT], space, space, done[MOUTH_RIGHT] );

}
#endif
