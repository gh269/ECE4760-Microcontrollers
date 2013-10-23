#ifndef __4760_DDS_H_
#define __4760_DDS_H__

//Each tone burst should ramp up to full amplitude over a time of about 
//5 milliseconds and ramp down again in about the same amount of time
//each button press is 1 sec
#define RAMPUPEND 250 // = 4*62.5 or 4mSec * 62.5 samples/mSec NOTE:max=255
#define RAMPDOWNSTART 625 // = 10*62.5
#define RAMPDOWNEND 875 // = 14*62.5 NOTE: RAMPDOWNEND-RAMPDOWNSTART<255 

#define INCREMENT_1209 83081271L
#define INCREMENT_1336 91808584L
#define INCREMENT_1477 101497963L

#define INCREMENT_697 47897143L
#define INCREMENT_770 52913630L
#define INCREMENT_852 58548588L
#define INCREMENT_941 64664579L
#define COUNTMS 62

#define NoPush 1 
#define MaybePush 2
#define Pushed 3
#define MaybeNoPush 4

unsigned char PushState;	//state machine  

//FSM button duration
volatile char count;
//Flags to mark that we're playing a tone now
volatile char is_timed_playing;
volatile char is_playing;
//Counters for timing
volatile int dds_duration;
volatile unsigned int time;
volatile unsigned int time1;

//play stop and play start are flags to mark that 
//we're starting (stoping) our tone generation and need to 
//ramp up (down) rampCount indexes into the ramping table
volatile char play_stop;
volatile char play_start;
volatile unsigned int rampCount
void update_status_variables();
void init_dtmf();
/*
fa is the high frequency
fb is the low frequency
play sets the accumulator increment values for 
requested frequencies
timed play calls play and also sets a duration for
timed tone generation (silence, # button playback)
*/
void play( int fA, int fB);
void timed_play(int fA, int fB, int duration);
void stop_playing();



#endif
