#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(){
	signed char sineTable[256];
	char rampTable[256];

	for( i = 0; i < 256; i++){
		/* 				                               i 
			(char)  127 * sin( [2pi * 1Hz] * (float)-------)
													  256
					t goes from 0 to 1

		*/
		sineTable[i] = (char)( 127.0 * sin( 6.283 * ((float)i) / 256.0) );
		//the following table needs
		//rampTable[0] = 0 and rampTable[255] = 127
		rampTable[i] = i >> 1;
	}


}