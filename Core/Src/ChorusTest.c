#include "main.h"

#define ECHO_GAIN 0.5
#define PI 3.14159f
#define NUM_SAMPLES 4096

//Sample rate and Output freq
float F_SAMPLE = 48000.0;
float F_OUT	= 1000.0;

extern uint8_t samples[NUM_SAMPLES];
extern CallBack_Result_t cb_result;

uint8_t caveBuff[4096];
int i = 0;
int chorusEn = 0;


void chorusEffect(){
	i=0;
	while(chorusEn) {
//		if(cb_result == FULL_COMPLETED) {
			caveBuff[i] = samples[i] + samples[i+1]*ECHO_GAIN;
			i = (i+1) % NUM_SAMPLES;
	//	}
	}
}