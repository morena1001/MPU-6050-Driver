/*
 * MPU6050.c
 *
 *  Created on: Sep 2, 2024
 *      Author: josue
 */

#include "MPU6050.h"

/*
 * INITIALIZATION
 */

uint8_t MPU6050_Init(MPU6050* device, I2C_HandleTypeDef* i2c_handle) {

	// Set struct parameters
	device->i2c_handle;

	for (int i = 0; i < 3; i++) {
		device->acc_g[i] = 0.0f;
		device->gyr_dps[i] = 0.0f;
	}

	device->temp_C = 0.0f;

	// Store number of transaction errors ( to be returned at the end of the function )
	uint8_t err_num = 0;
	HAL_StatusTypeDef status;

	// Check WHO_AM_I register (RM p.46)
	uint8_t reg_data;

	status = MPU6050_ReadRegister(device, MPU6050_REG_WHO_AM_I, &reg_data);
	err_num += (status != HAL_OK);

	if (reg_data != WHO_AM_I)	return 255;

	// CONFIG : Set external Frame Synchronization (FSYNC) and Digital Low Pass Filter (DLPF)
	// FSYNC input disabled, Accelerometer : 184Hz bandwidth, 2.0ms delay, Gyroscope : 188Hz bandwidth, 1.9ms delay, 1kHz Fs
	reg_data = 0x01;

	status = MPU6050_WriteRegister(device, MPU6050_REG_CONFIG, &reg_data);
	err_num += (status != HAL_OK);

	// GRYO_CONFIG : Enable gyroscope self-test (XG_ST, YG_ST, ZG_ST) and set full scale range (FS_SEL)
	// Self-tests are disabled [ MOMENTARILY ], full scale range +- 500 deg/s
	reg_data = 0x08;

	status = MPU6050_WriteRegister(device, MPU6050_REG_GYRO_CONFIG, &reg_data);
	err_num += (status != HAL_OK);

	// ACCEL_CONFIG : Enable accelerometer self-test (XA_ST, YA_ST, ZA_ST), and set full scale range (AFS_SEL)
	// Self-tests are disabled [ MOMENTARILY ], full scale range +- 4g
	reg_data = 0x08;

	status = MPU6050_WriteRegister(device, MPU6050_REG_ACCEL_CONFIG, &reg_data);
	err_num += (status != HAL_OK);

	// SLAVE 0 CONTROL

	// INT_PIN_CFG : Set behavior of interrupt signal in INT pin
	// Logic level is active high, pin is push-pull configured, pin is held high until interrupt is cleared,
	// Interrupt status bits are cleared by reading INT_STATUS (REGISTER 3A), FSYNC_INT_LEVEL is active high,
	// FSYNC_INT_EN disabled FSYNC pin from causing interrupt, I2C_BYPASS_EN does not allow host from accessing auxiliary I2C bus
	reg_data = 0x20;

	status = MPU6050_WriteRegister(device, MPU6050_REG_INT_PIN_CFG, &reg_data);
	err_num += (status != HAL_OK);

	// PWR_MGMT_1 : Set power mode, clock source, and temperature sensor
	// Device is not reset on startup, sleep is disabled and clock is enabled, the temperature sensor is enabled
	// The clock source is set to the internal 8MHz oscillator
	reg_data = 0x20;

	status = MPU6050_WriteRegister(device, MPU6050_REG_PWR_MGMT_1, &reg_data);
	err_num += (status != HAL_OK);

	// PWR_MGMT_2 : Set frequency of wake-ups in AOLPM, and set axes of gyroscope and accelerometer to standby mode
	// Wake-up frequency set to 5hz, no axes put into standby mode
	reg_data = 0x40;

	status = MPU6050_WriteRegister(device, MPU6050_REG_PWR_MGMT_2, &reg_data);
	err_num += (status != HAL_OK);

	// Return number of errors (0 if successful initialization)
	return err_num;
}



/*
 * DATA AQUISITION
 */

HAL_StatusTypeDef MPU6050_ReadAccelerometer(MPU6050* device) {

}

HAL_StatusTypeDef MPU6050_ReadGyroscope(MPU6050* device) {

}

HAL_StatusTypeDef MPU6050_ReadTemperature(MPU6050* device) {

}



/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050* device, uint8_t reg, uint8_t* data) {
	return HAL_I2C_Mem_Read(device->i2c_handle, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050* device, uint8_t reg, uint8_t* data, uint8_t length) {
	return HAL_I2C_Mem_Read(device->i2c_handle, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050* device, uint8_t reg, uint8_t* data) {
	return HAL_I2C_Mem_Write(device->i2c_handle, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}


