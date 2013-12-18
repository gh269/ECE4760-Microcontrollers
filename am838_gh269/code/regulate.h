#ifndef REGULATE_H
#define REGULATE_H

volatile int cTemp;		// current temperature
volatile int dTemp;		// desired temperature
volatile int time_rem;  // time remaining in seconds

// timer variable
volatile int msec;
volatile int count_en;
volatile int beep_timer;
volatile int adjust_temp_timer;
// relay & LED bits
#define LED_EN 	 0x04
// PORT D3
#define RELAY_EN 0x08
//speaker D4
#define SOUND_EN 0x10
#define SEM_SHARED 4
#define BEEP_ONCE_TIME 600
#define ADJUST_PERIOD 2000

//PID control terms
#define PROP 0.07
#define INTEG 0.04
#define DIFF 0.05

struct PID_DATA pid_data;

#endif 