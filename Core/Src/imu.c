#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "Fusion.h"
#include "main.h"

#define MPU6050_ADDR 0x68
#define REG_POWER 0x6B
#define REG_WHO_AM_I 0x75
#define REG_SMPLRT_DIV 0x19
#define REG_ACCEL_CONF 0x1B
#define REG_GYRO_CONF 0x1C
#define REG_ACCLE_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43

struct MPU6050_ACCLE_t {
	float AccelX;
	float AccelY;
	float AccelZ;
};

extern I2C_HandleTypeDef hi2c1;

void initIMU(void) {
	uint8_t who;
	uint8_t data;
	
	//Check I2C device address
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_WHO_AM_I, 1, &who, 1, 1000);
	
	if(who = 0x75) {
		data = 0;
		//Send Wakeup
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_POWER, 1, &data, 1, 1000);
		
		//Sample rate divider
		//Sample Rate = 1Khz --> SMPLRT_DIV = 7
		data = 0x07
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_SMPLRT_DIV, 1, &data, 1, 1000);
		
		//+- 4g
		data = 0x01;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_ACCEL_CONF, 1, &data, 1, 1000);
		
		//+- 500 deg/s
		data = 0x01;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_GYRO_CONF, 1, &data, 1, 1000);
	}
}

void imuRead(MPU6050_Accel *accel, MPU6050_Gyro *gyro) {
	uint8_t accelData[6];
	
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_ACCLE_XOUT_H, 1, accelData, 6, 1000);
	
	int16_t Accel_X_Raw = (int16_t)()
	
	
}



