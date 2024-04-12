
#include "main.h"

//#include "hal_config.h"


#define BUFF_SIZE 16000
#define ECHO_GAIN 0.5
#define PI 3.14159f
#define NUM_SAMPLES 4096

//Sample rate and Output freq
float F_SAMPLE = 48000.0;
float F_OUT	= 1000.0;


extern uint8_t samples[NUM_SAMPLES];


uint8_t caveBuff[BUFF_SIZE];
int8_t tail = 0;
int i = 0;

int8_t out_sample = 0;
int8_t in_sample = 0;

//int bop = samples[i];


//void ProcessData(uint8_t samples){
///*buffer operation*/
//caveBuff[i] = samples[i] + 
//	
//	
//	
//}