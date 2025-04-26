/*
 * mpu6050.c
 *
 *  Created on: Apr 24, 2025
 *      Author: danilo
 */

#include "mpu6050.h"
#include <stdio.h>


static uint8_t mpu6050_i2c_addr;

//single byte read sequence
mpu6050_status_t mpu6050_read_byte(I2C_HandleTypeDef *hi2c, uint8_t reg_address, uint8_t *data){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, mpu6050_i2c_addr << 1, reg_address, 1, data, 1, I2C_TIMEOUT);
	return (status == HAL_OK) ? MPU6050_OK : MPU6050_ERROR;
}

//burst read sequence
mpu6050_status_t mpu6050_read(I2C_HandleTypeDef *hi2c, uint8_t reg_base_address, uint8_t *data, uint32_t n_bytes){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, mpu6050_i2c_addr << 1, reg_base_address, 1, data, n_bytes, I2C_TIMEOUT);
	return (status == HAL_OK) ? MPU6050_OK : MPU6050_ERROR;
}


mpu6050_status_t mpu6050_write_byte(I2C_HandleTypeDef *hi2c, uint8_t reg_address, uint8_t data){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, mpu6050_i2c_addr << 1, reg_address, 1, &data, 1, I2C_TIMEOUT);
	return (status == HAL_OK) ? MPU6050_OK : MPU6050_ERROR;
}


mpu6050_status_t mpu6050_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_address){
	mpu6050_i2c_addr = i2c_address;

	uint8_t read_byte = 0;
	if(mpu6050_read_byte(hi2c, MPU6050_REG_WHOAMI, &read_byte) != MPU6050_OK){
		return MPU6050_ERROR;
	}
	if(read_byte == 0x68 || read_byte == 0x98){
		printf("Valid MPU6050 at %X\n", mpu6050_i2c_addr);
	}else{
		printf("Invalid device at %X\n", mpu6050_i2c_addr);
		return MPU6050_ERROR;
	}

	uint8_t data = 0x00;
	if(mpu6050_write_byte(hi2c, MPU6050_REG_PWMGM_1, data) != MPU6050_OK){
		return MPU6050_ERROR;
	}
	return MPU6050_OK;
}


mpu6050_status_t mpu6050_read_accelerometer_data(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr, mpu6050_accel_data_t* accel_data){
	uint8_t data[6];
	mpu6050_status_t status = mpu6050_read(hi2c, MPU6050_REG_ACCEL_START, data, sizeof(data));
	if(status != MPU6050_OK){
		return status;
	}
	accel_data->x = (int16_t)((data[0] << 8) | (data[1]));
	accel_data->y = (int16_t)((data[2] << 8) | (data[3]));
	accel_data->z = (int16_t)((data[4] << 8) | (data[5]));
	return MPU6050_OK;
}

mpu6050_accel_data_t mpu6050_accel_calibration(const mpu6050_accel_data_t *error_offset, mpu6050_accel_data_t *raw_data){
	mpu6050_accel_data_t accel_calibrated;
	accel_calibrated.x = raw_data->x - error_offset->x;
	accel_calibrated.y = raw_data->y - error_offset->y;
	accel_calibrated.z = raw_data->z - error_offset->z;

	return accel_calibrated;
}

mpu6050_status_t mpu6050_configure_lpf(I2C_HandleTypeDef *hi2c, mpu6050_dlpf_config_t dlpf){
	uint8_t value = 0;

	if(mpu6050_read_byte(hi2c, MPU6050_REG_CONFIG, &value) != MPU6050_OK){
		return MPU6050_ERROR;
	}
	value &= ~(0x7);
	value |= (uint8_t)dlpf;
	if(mpu6050_write_byte(hi2c, MPU6050_REG_CONFIG, value) != MPU6050_OK){
		return MPU6050_ERROR;
	}
	return MPU6050_OK;
}
