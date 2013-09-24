#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "dds.h"

int main(){
	signed char sineTable[256];
	char rampTable[256];
	unsigned int i;

	char rampTablePrime[312];

	float current = 0;
	float start = 0;
	float end = 127;
	float totalSteps = 311;
	float step = (end - start) / totalSteps;

	for(i = 0; i < 312; i++){
		rampTablePrime[i] = (int)(current);
		current += step;
	}

	// for( i = 0; i < 312; i++){
	// 	/* 				                               i 
	// 		(char)  127 * sin( [2pi * 1Hz] * (float)-------)
	// 												  256
	// 				t goes from 0 to 1

	// 	*/
	// 	//sineTable[i] = (char)( 127.0 * sin( 6.283 * ((float)i) / 256.0) );
	// 	//the following table needs
	// 	//rampTable[0] = 0 and rampTable[255] = 127
	// 	rampTable[i] = i >> 1;
	// }
	printf("{ \n");

	for( i = 0; i < 312; i++){
		printf("%d, ", rampTablePrime[i]);
		if( i % 12 == 0 && i != 0){
			printf("\n");
		}

	}
	printf(" }\n");



	//----------------------------------
	//------CALCULATE INCREMENTS--------
	//----------------------------------

	//unsigned long increment;
	/*	     M
	Fout=	--- Fclk
			2^N

	     2^N * Fout 
	 M = ---------- ; Fout = 1000
	        Fclk

	M = 68719 * fout
	*/
}