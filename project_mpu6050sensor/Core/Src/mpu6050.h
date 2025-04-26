/*
 * mpu6050.h
 *
 *  Created on: Apr 24, 2025
 *      Author: danilo
 */

#ifndef SRC_MPU6050_H_
#define SRC_MPU6050_H_
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef enum {
	MPU6050_OK,
	MPU6050_ERROR
}mpu6050_status_t;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}mpu6050_accel_data_t;

typedef enum {
	DLPF_CFG_260HZ = 0,
	DLPF_CFG_184HZ = 1,
	DLPF_CFG_94HZ = 2,
	DLPF_CFG_44HZ = 3,
	DLPF_CFG_21HZ = 4,
	DLPF_CFG_10HZ = 5,
	DLPF_CFG_5HZ = 6,
}mpu6050_dlpf_config_t;

mpu6050_status_t mpu6050_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_address);
mpu6050_status_t mpu6050_read_accelerometer_data(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr, mpu6050_accel_data_t* accel_data);
mpu6050_accel_data_t mpu6050_accel_calibration(const mpu6050_accel_data_t *error_offset, mpu6050_accel_data_t *raw_data);
mpu6050_status_t mpu6050_configure_lpf(I2C_HandleTypeDef *hi2c, mpu6050_dlpf_config_t dlpf);
#define I2C_TIMEOUT                      500UL

//registers
#define MPU6050_REG_WHOAMI               (uint8_t)117
#define MPU6050_REG_PWMGM_1              (uint8_t)107
#define MPU6050_REG_ACCEL_START          (uint8_t)59
#define MPU6050_REG_CONFIG               (uint8_t)26

#endif /* SRC_MPU6050_H_ */
