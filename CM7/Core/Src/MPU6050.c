/*
 * MPU6050.c
 *
 *  Created on: Nov 16, 2021
 *      Author: jorge
 */

#include "MPU6050.h"

void MPU6050_init(I2C_HandleTypeDef * hi2c, MPU6050* mpu){
	uint8_t check; // check data
	uint8_t data; //send data

	HAL_I2C_Mem_Read(hi2c,  mpu->direction , mpu->who_am_I, 1, &check, 1, mpu->timeout);
	if(check == 104){ // WHO_I_AM Register default value

		// Configure clock to 8MHZ oscilator
		data = 0x00;
		HAL_I2C_Mem_Write(hi2c, mpu->direction, PWR_MGMT_1, 1, &data, 1, mpu->timeout);
		/* Sample rate = 8khz/(1+SMPLRT_DIV)
		 * Send 7 to SMPLRT_DIV register to obtain 1khz sample rate
		 * */
		data = 0x07;
		HAL_I2C_Mem_Write(hi2c, mpu->direction , SMPLRT_DIV, 1, &data, 1, mpu->timeout);

		// Configure gyroscope Full scale range
		// +-250 °/s => FS_SEL = 0
		data = 0x00;
		HAL_I2C_Mem_Write(hi2c, mpu->direction , GYRO_CONFIG, 1, &data, 1, mpu->timeout);
		// cONFIGURE ACCELEROMETER
		// +-2 G => AFS_SEL = 0
		data = 0x00;
		HAL_I2C_Mem_Write(hi2c, mpu->direction , ACCEL_CONFIG, 1, &data, 1, mpu->timeout);

		for(int i = 0;i < 5;i++){ // 5 Blink confirmation
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_Delay(500);
		}

	}
}

float MPU6050_getTemp(I2C_HandleTypeDef * hi2c, MPU6050* mpu){

	uint8_t buffer[2];
	float temp = 0;
	int16_t temp_raw;
	HAL_I2C_Mem_Read(hi2c, mpu->direction, TEMP_OUT_H, 1, &buffer, 2, mpu->timeout);
	temp_raw = (int16_t)(buffer[0] << 8 | buffer[1]);
	temp = (temp_raw/340.0)+36.53;
	return temp;

}

void MPU6050_get_acc(I2C_HandleTypeDef * hi2c, MPU6050* mpu, float* p2Strg){

    uint8_t buffer[6];
	int16_t ax_raw, ay_raw, az_raw;

	HAL_I2C_Mem_Read(hi2c, mpu->direction, ACCEL_XOUT_H, 1, &buffer, 6, mpu->timeout);
        // Physically, the information is stored in bytes on the MPU
            // So... we merge two bytes into 1 16bits int
    ax_raw = (int16_t)(buffer[0] << 8 | buffer[1]);
	ay_raw = (int16_t)(buffer[2] << 8 | buffer[3]);
	az_raw = (int16_t)(buffer[4] << 8 | buffer[5]);

	p2Strg [0] = ax_raw/16384.0;
	p2Strg [1] = ay_raw/16384.0;
	p2Strg [2] = az_raw/16384.0;

	//return accReading;

}

void MPU6050_get_gyro(I2C_HandleTypeDef * hi2c, MPU6050* mpu, float* p2Strg){
	uint8_t buffer[6];
	int16_t gx_raw, gy_raw, gz_raw;
	//float gyro[3];
		/*The structure for reading:
            gyro[0] = gx
            gyro[1] = gy
            gyro[2] = gz
        */
	HAL_I2C_Mem_Read(hi2c, mpu->direction, GYRO_XOUT_H, 1, buffer, 6, mpu->timeout);
	gx_raw = (int16_t)(buffer[0] << 8 | buffer [1]);
	gy_raw = (int16_t)(buffer[2] << 8 | buffer [3]);
	gz_raw = (int16_t)(buffer[4] << 8 | buffer [5]);

	p2Strg [0] = gx_raw/131.0;
	p2Strg [1] = gy_raw/131.0;
	p2Strg [2] = gz_raw/131.0;

	//return gyro;

}
