#ifndef __4760_DDS_H_
#define __4760_DDS_H__

/*

Dependencies:

Uses Timer0 for the DDS
Pin B3 is the Timer0 OC0A output

*/

//Each tone burst should ramp up to full amplitude over a time of about 
//5 milliseconds and ramp down again in about the same amount of time
//each button press is 1 sec

#define RAMPUPEND  312//     5 ms * 62.5 cycles / ms
#define RAMPDOWNSTART 62187// (1000 ms - 5 = 995) ms * 62.5 cycles / ms 
#define RAMPDOWNEND 62500 // 1000 * 62.5

//increment = 68719 * F
#define INCREMENT_1209 83081271L
#define INCREMENT_1336 91808584L
#define INCREMENT_1477 101497963L

#define INCREMENT_697 47897143L
#define INCREMENT_770 52913630L
#define INCREMENT_825 56693175L
#define INCREMENT_941 646664579L

char is_playing;
volatile int dds_duration;
volatile unsigned int time;
volatile unsigned int time1;

void init_dtmf();
/*
fa is the high frequency
fb is the low frequency
*/
void play( int fA, int fB, int time);
void stop_playing();
#endif
