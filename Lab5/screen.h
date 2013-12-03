#ifndef SCREEN_H
#define SCREEN_H


#define H_IDX 0
#define T_IDX 1
#define EXCL_IDX 2

#define MOUTH_LEFT 0
#define MOUTH_RIGHT 1

//B0 0000 0001
#define CLK 0x01
//B1 0000 0010
#define CLR 0x02

// B2 0000 0100
#define SCREEN_LEFT 0x04
//B3 0000 1000
#define SCREEN_RIGHT 0x08

#define PULSE_DELAY 1
#define ROW_SCAN_DELAY 250

// row scan release = 8 * row_scan_delay + fuzz 
// converted into seconds 
// 8 * 3000 ms -> 0.024 seconds 
// 0.03 
#define ROW_SCAN_REL 0.03

void init_screens(){
	DDRB = (CLK | CLR | SCREEN_LEFT | SCREEN_RIGHT);
	PORTB = CLR;
}

char clock_value;

int displaybuffer_left[] = { 0,
					0,
					0,
					0,
					0,
					0,
					0,
					0
				 };

int displaybuffer_right[] = { 0,
					0,
					0,
					0,
					0,
					0,
					0,
					0
				 };
				 // 0001 0000, 
int row_order[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};

int numbers[][8] = {
	// 1   2    3    4    5    6    7    8
	{0xf, 0x1, 0x5, 0x5, 0x5, 0x5, 0x1, 0xf}, //0
	{0xf, 0xd, 0xd, 0xd, 0xd, 0x9, 0xd, 0xf}, //1
	{0xf, 0x1, 0x7, 0x7, 0x1, 0xd, 0x1, 0xf}, //2
	{0xf, 0x1, 0xd, 0xd, 0x9, 0xd, 0x1, 0xf}, //3
	{0xf, 0xd, 0xd, 0xd, 0x1, 0x5, 0x5, 0xf}, //4
	{0xf, 0x1, 0xd, 0xd, 0x1, 0x7, 0x1, 0xf}, //5
	{0xf, 0x1, 0x5, 0x1, 0x7, 0x7, 0x1, 0xf}, //6
	{0xf, 0xb, 0xb, 0xb, 0xb, 0xd, 0x1, 0xf}, //7
	{0xf, 0x1, 0x5, 0x5, 0x1, 0x5, 0x1, 0xf}, //8
	{0xf, 0xd, 0xd, 0xd, 0x1, 0x5, 0x1, 0xf}, //9
};

int hot[][8] = {
	//1   2    3     4    5   6    7   8
	{0xf, 0x5, 0x5, 0x1, 0x5, 0x5, 0x5, 0xf}, //H
	{0xf, 0xb, 0xb, 0xb, 0xb, 0xb, 0x1, 0xf}, //T
	{0xf, 0xb, 0xf, 0xb, 0xb, 0xb, 0xb, 0xf}  //!

};

int minutes[] =   { 0xf, 0xa, 0xa, 0x8, 0x8, 0xf, 0xf, 0xf};
int seconds[] =   { 0xf, 0x8, 0xe, 0x8, 0xb, 0x8, 0xf, 0xf};
int space[] =     { 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf};

int colon[] = { 0xf, 0xf, 0xb, 0xf, 0xf, 0xb, 0xf, 0xf};
int deg_celsius[] = { 0xf, 0x9, 0xb, 0x9, 0xf, 0x3, 0x3, 0xf};

int mouth[][8] = {
	//1   2    3     4    5   6    7   8
	{0xf, 0xf, 0xf, 0x8, 0xb, 0x0, 0xf, 0xf}, //Mouth Left
	{0xf, 0xf, 0xf, 0x1, 0xd, 0x0, 0xf, 0xf}  //Mouth Right
};

int done[][8] = {
	//1   2    3     4    5   6    7   8
	{0xf, 0xe, 0xd, 0xd, 0xd, 0xe, 0xf, 0xf}, //Done Left
	{0xf, 0x7, 0xb, 0xb, 0xb, 0x7, 0xf, 0xf}  //Done Right
};

//toggles pin in ISR w/ 1us time base
void pulse_out();
void write_to_buffer(int * left_valueR, int * left_valueL, int * right_valueR, int * right_valueL);
void write_buffers_to_screen();
void write_two_values_to_buffer(int * valueR, int * valueL, int screen);
//void delayMicroseconds(int delay);
void delay_row_scan(int delay);

void write_happy_to_buffer();
void write_empty_to_buffer();
void write_hot_to_buffer();
void write_temp_to_buffer(int temp);
void write_min_to_buffer(int min);
void write_sec_to_buffer(int sec);
void write_done_to_buffer();



#endif
