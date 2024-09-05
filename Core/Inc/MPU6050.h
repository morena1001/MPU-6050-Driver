/*
 * MPU6050.h
 *
 *  Created on: Sep 2, 2024
 *      Author: josue
 *
 *  PS is the Product Specification document
 *  RM is the Register Map document
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f3xx_hal.h" // NEEDED FOR I2C

/*
 * DEFINES
 */

#define MPU6050_I2C_ADDR	(0x68 << 1) // AD0 = 0 -> 0x68, AD0 = 1 -> 0x69 (PS p.15)

#define WHO_AM_I 			0x68

//#define READ_BIT 		  	1
//#define WRITE_BIT 			0

/*
 * REGISTERS (RM p.6 - 8)
 */

#define MPU6050_REG_SELF_TEST_X 		0x0D
#define MPU6050_REG_SELF_TEST_Y 		0x0E
#define MPU6050_REG_SELF_TEST_Z 		0x0F
#define MPU6050_REG_SELF_TEST_A 		0x10
#define MPU6050_REG_SMPLRT_DIV			0x19
#define MPU6050_REG_CONFIG				0x1A
#define MPU6050_REG_GYRO_CONFIG			0x1B
#define MPU6050_REG_ACCEL_CONFIG		0x1C
#define MPU6050_REG_MOT_THR				0x1F
#define MPU6050_REG_FIFO_EN				0x23
#define MPU6050_REG_I2C_MST_CTRL		0x24
#define MPU6050_REG_I2C_SLV0_ADDR		0x25
#define MPU6050_REG_I2C_SLV0_REG		0x26
#define MPU6050_REG_I2C_SLV0_CTRL		0x27
#define MPU6050_REG_I2C_SLV1_ADDR		0x28
#define MPU6050_REG_I2C_SLV1_REG		0x29
#define MPU6050_REG_I2C_SLV1_CTRL		0x2A
#define MPU6050_REG_I2C_SLV2_ADDR		0x2B
#define MPU6050_REG_I2C_SLV2_REG		0x2C
#define MPU6050_REG_I2C_SLV2_CTRL		0x2D
#define MPU6050_REG_I2C_SLV3_ADDR		0x2E
#define MPU6050_REG_I2C_SLV3_REG		0x2F
#define MPU6050_REG_I2C_SLV3_CTRL		0x30
#define MPU6050_REG_I2C_SLV4_ADDR		0x31
#define MPU6050_REG_I2C_SLV4_REG		0x32
#define MPU6050_REG_I2C_SLV4_DO			0x33
#define MPU6050_REG_I2C_SLV4_CTRL		0x34
#define MPU6050_REG_I2C_SLV4_DI			0x35
#define MPU6050_REG_I2C_MST_STATUS		0x36
#define MPU6050_REG_INT_PIN_CFG			0x37
#define MPU6050_REG_INT_ENABLE			0x38
#define MPU6050_REG_INT_STATUS			0x3A
#define MPU6050_REG_ACCEL_XOUT_H		0x3B
#define MPU6050_REG_ACCEL_XOUT_L		0x3C
#define MPU6050_REG_ACCEL_YOUT_H		0x3D
#define MPU6050_REG_ACCEL_YOUT_l		0x3E
#define MPU6050_REG_ACCEL_ZOUT_H		0x3F
#define MPU6050_REG_ACCEL_ZOUT_L		0x40
#define MPU6050_REG_TEMP_OUT_H			0x41
#define MPU6050_REG_TEMP_OUT_L			0x42
#define MPU6050_REG_GYRO_XOUT_H			0x43
#define MPU6050_REG_GYRO_XOUT_L			0x44
#define MPU6050_REG_GYRO_YOUT_H			0x45
#define MPU6050_REG_GYRO_YOUT_L			0x46
#define MPU6050_REG_GYRO_ZOUT_H			0x47
#define MPU6050_REG_GYRI_ZOUT_L			0x48
#define MPU6050_REG_I2C_SLV0_DO			0x63
#define MPU6050_REG_I2C_SLV1_DO			0x64
#define MPU6050_REG_I2C_SLV2_DO			0x65
#define MPU6050_REG_I2C_SLV3_DO			0x66
#define MPU6050_REG_I2C_MST_DELAY_CTRL	0x67
#define MPU6050_REG_SIGNAL_PATH_RESET	0x68
#define MPU6050_REG_MOT_DETECT_CTRL		0x69
#define MPU6050_REG_USER_CTRL			0x6A
#define MPU6050_REG_PWR_MGMT_1			0x6B
#define MPU6050_REG_PWR_MGMT_2			0x6C
#define MPU6050_REG_FIFO_COUNTH			0x72
#define MPU6050_REG_FIFO_COUNTL			0x73
#define MPU6050_REG_FIFO_R_W			0x74
#define MPU6050_REG_WHO_AM_I			0x75

/*
 * SENSOR STRUCT
 */

typedef struct {
	// I2C handle
	I2C_HandleTypeDef* i2c_handle;

	// Accelerometer data (X, Y, Z) in g
	float acc_g[3];

	// Gyroscope data (x, y, z) in deg/s
	float gyr_dps[3];

	// Temperature data in deg
	float temp_C;
} MPU6050;

/*
 * INITIALIZATION
 */

uint8_t MPU6050_Init(MPU6050* device, I2C_HandleTypeDef* i2c_handle); // Outputs possible error codes

/*
 * DATA AQUISITION
 */

HAL_StatusTypeDef MPU6050_ReadAccelerometer(MPU6050* device);
HAL_StatusTypeDef MPU6050_ReadGyroscope(MPU6050* device);
HAL_StatusTypeDef MPU6050_ReadTemperature(MPU6050* device);

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050* device, uint8_t reg, uint8_t* data);
HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050* device, uint8_t reg, uint8_t* data, uint8_t length);

HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050* device, uint8_t reg, uint8_t* data);

/*
 * REGISTER ID'S
 * DEVICE ID'S
 * DRIVER FUNCTION
 */



#endif /* INC_MPU6050_H_ */
