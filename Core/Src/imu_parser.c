#include "main.h"
#include "imu_parser.h"
#include <stdbool.h>
#include "stm324xg_eval_io.h"
#include "stm324xg_eval_lcd.h"
#include "mpu6050.h"
#include "voxart_dev.h"
#include <stdio.h>
#include <stdlib.h>

/*********************** */
/*    Extern Variables   */ 
/*********************** */
extern MPU6050_t MPU6050;
extern imuMovement imu;
extern TIM_HandleTypeDef htim13;
extern uint32_t timestamp;

enum effectState state = NONE;

void setStates() {
	if(MPU6050.KalmanAngleX > -10 && MPU6050.KalmanAngleX < 10 && MPU6050.KalmanAngleY > -10 && MPU6050.KalmanAngleY < 10 && state != PITCH) {
		state = NONE;
	} else if(MPU6050.KalmanAngleX > -10 && MPU6050.KalmanAngleX < 6 && MPU6050.KalmanAngleY > 100 && MPU6050.KalmanAngleY < 120 && state != PITCH) {
		state = REVERB;
	} else if(MPU6050.KalmanAngleX > -5 && MPU6050.KalmanAngleX < 5 && MPU6050.KalmanAngleY < -175 && MPU6050.KalmanAngleY > -181 && state != PITCH) {
		state = CHORUS;
	}
}

void parseReverbData() {
	if (MPU6050.Az > 1.2) {
		//serialPrintln("Positive X");
		imu.x = 1;
		imu.l = POS;
	}
	
	if(MPU6050.Az < -1.5 && imu.l != POS) {
		//serialPrintln("Negative X");
		imu.x = -1;
	} 
	if(MPU6050.Az < 1.2 && MPU6050.Az > -1) {
		imu.x = 0;
		imu.l = NO;
	}
}

void parseChorusData() {
	if(MPU6050.KalmanAngleX < -10) {
		imu.y = 1;
	}
	if(MPU6050.KalmanAngleX > 25) {
		imu.y = -1;
	}
	if(MPU6050.KalmanAngleX > -5 && MPU6050.KalmanAngleX < 25) {
		imu.y = 0;
	}
}



