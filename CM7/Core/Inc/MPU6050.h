/*
 * MPU6050.h
 *
 *  Created on: Oct 14, 2021
 *      Author: jorge
 */

#ifndef SRC_MPU6050_H_
#define SRC_MPU6050_H_

// To be able to use I2C handler
#include "stm32h7xx_hal.h"

// Gyro Registers
#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0X1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B   // Initial reg where acc is stored
#define GYRO_XOUT_H 0x43    // Initial reg where gyro is stored
#define TEMP_OUT_H 0x41     // Initial reg where acc temp stored
#define PWR_MGMT_1 0x6B

typedef struct {
    int who_am_I;    // Default: 0x75
    int direction;   // Default: 0xD0
    int timeout;     // Default: 1000
} MPU6050;

/*Initializator, kinda contructor*/
void MPU6050_init(I2C_HandleTypeDef* hi2c, MPU6050* mpu);

void MPU6050_get_acc(I2C_HandleTypeDef* hi2c, MPU6050* mpu, float* p2Strg);

void MPU6050_get_gyro(I2C_HandleTypeDef* hi2c, MPU6050* mpu, float* p2Strg);

float MPU6050_getTemp(I2C_HandleTypeDef* hi2c, MPU6050* mpu);

#endif /* SRC_MPU6050_H_ */
