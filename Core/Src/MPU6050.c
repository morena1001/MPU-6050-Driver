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
	device->i2c_handle = i2c_handle;

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

	// INT_ENABLE : Sets interrupt generation
	// Only the data ready interrupt is enabled
	reg_data = 0x01;

	status = MPU6050_WriteRegister(device, MPU6050_REG_INT_ENABLE, &reg_data);
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
	// (RM p.30)

	// Read raw values from accelerometer registers
	uint8_t reg_data[6];

	HAL_StatusTypeDef status = MPU6050_ReadRegisters(device, MPU6050_REG_ACCEL_XOUT_H, reg_data, 6);

	// Combine register values to give raw (unsigned) accelerometer values
	int16_t raw_signed_data[3];

	raw_signed_data[0] = (reg_data[0] << 8) | reg_data[1]; // X-axis
	raw_signed_data[1] = (reg_data[2] << 8) | reg_data[3]; // Y-axis
	raw_signed_data[2] = (reg_data[4] << 8) | reg_data[5]; // Z-axis

	// Convert to g, Given range setting of +-4g
	device->acc_g[0] = 0.0001220703125 * raw_signed_data[0];
	device->acc_g[1] = 0.0001220703125 * raw_signed_data[1];
	device->acc_g[2] = 0.0001220703125 * raw_signed_data[2];

	return status;
}

HAL_StatusTypeDef MPU6050_ReadGyroscope(MPU6050* device) {
	// (RM p.32)

	// Read raw values from accelerometer registers
	uint8_t reg_data[6];

	HAL_StatusTypeDef status = MPU6050_ReadRegisters(device, MPU6050_REG_GYRO_XOUT_H, reg_data, 6);

	// Combine register values to give raw (unsigned) accelerometer values
	int16_t raw_signed_data[3];

	raw_signed_data[0] = (reg_data[0] << 8) | reg_data[1]; // X-axis
	raw_signed_data[1] = (reg_data[2] << 8) | reg_data[3]; // Y-axis
	raw_signed_data[2] = (reg_data[4] << 8) | reg_data[5]; // Z-axis

	// Convert to g, Given range setting of +-4g
	device->acc_g[0] = 0.0152671755725 * raw_signed_data[0];
	device->acc_g[1] = 0.0152671755725 * raw_signed_data[1];
	device->acc_g[2] = 0.0152671755725 * raw_signed_data[2];

	return status;
}

HAL_StatusTypeDef MPU6050_ReadTemperature(MPU6050* device) {
	// (RM p.31)

//	// The 16 bit TEMP_OUT value is signed, so the highest 8 bits are stored in a signed integer,
//	// And the lowest 8 bits are stored in an unsigned integer
//	int8_t reg_data_H;
//	uint8_t reg_data_L;
//
//	// Read the raw values of the two registers
//	HAL_StatusTypeDef status_H = MPU6050_ReadRegister(device, MPU6050_REG_TEMP_OUT_H, reg_data_H);
//	HAL_StatusTypeDef status_L = MPU6050_ReadRegister(device, MPU6050_REG_TEMP_OUT_L, reg_data_L);
//
//	// Combine register values to get raw temperature data
//	int16_t raw_data = (reg_data_H << 8) | reg_data_L;
//
//	// Convert to degrees Celsius
//	device->temp_C = ((float) raw_data / 340) + 36.53f;
//
//	// If any of the register readings did not result in a HAL_OK, return that staus, else return HAL_OK
//	if (status_H != HAL_OK) {
//		return status_H;
//	} else if (status_L != HAL_OK) {
//		return status_L;
//	} else {
//		return HAL_OK;
//	}

	// Read raw values from temperature registers
	uint8_t reg_data[2];

	HAL_StatusTypeDef status = MPU6050_ReadRegisters(device, MPU6050_REG_TEMP_OUT_H, reg_data, 2);

	// / Combine register values to get raw temperature data
	uint16_t raw_data = (reg_data[0] << 8) | reg_data[1];

	// Convert to degrees Celsius
	device->temp_C = ((float) raw_data / 340) + 36.53f;

	return status;
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


