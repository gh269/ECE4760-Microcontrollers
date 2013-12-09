#ifndef REGULATE_H
#define REGULATE_H

volatile int cTemp;		// current temperature
volatile int dTemp;		// desired temperature
volatile int time_rem;  // time remaining in seconds

// timer variable
volatile int msec;
volatile int count_en;
volatile int beep_timer;

//speaker D4
#define SOUND_EN 0x10
#define SEM_SHARED 4
#define BEEP_ONCE_TIME 600
#endif 