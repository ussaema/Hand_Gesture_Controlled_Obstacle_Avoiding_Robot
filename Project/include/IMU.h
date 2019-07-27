/*
* IMU.h
*
* Created: 07.07.2019 10:23:03
* Author: ussaema
*/


#ifndef __IMU_H__
#define __IMU_H__

#define RADIANS_TO_DEGREES 180/3.14159
#define GYROGAIN 16.4
#define ACCGAIN 16384.0
#define ACCGYRO_ADDR 0x3B
#define IMU_PWR_MGMT_1 0x6B
#define IMU_PWR1_SLEEP_BIT 6
#define IMU_PWR1_CLKSEL_BIT 2
#define IMU_PWR1_CLKSEL_LENGTH 3
#define IMU_CLOCK_PLL_XGYRO 0x01
#define IMU_CONFIG 0x1A
#define IMU_CFG_DLPF_CFG_BIT 2
#define IMU_CFG_DLPF_CFG_LENGTH 3
#define IMU_DLPF_BW_42 0x03
#define IMU_GYRO_CONFIG 0x1B
#define IMU_GCONFIG_FS_SEL_BIT 4
#define IMU_GCONFIG_FS_SEL_LENGTH 2
#define IMU_GYRO_FS_2000 0x03
#define IMU_ACCEL_CONFIG 0x1C
#define IMU_ACONFIG_AFS_SEL_BIT 4
#define IMU_ACONFIG_AFS_SEL_LENGTH 2
#define IMU_SMPLRT_DIV 0x19
#define IMU_WHO_AM_I 0x75
#define IMU_WHO_AM_I_BIT 6
#define IMU_WHO_AM_I_LENGTH 6

#include "Kalman.h"
#include "UART.h"
#include "MCU.h"
#include "i2cmaster.h"

#include <util/delay.h>

class IMU
{
	//variables
	public:
	protected:
	private:
	volatile uint8_t buffer[14];
	uint8_t _address;
	uint8_t _pin_led;
	
	float dt; unsigned long t0, t1;
	
	int16_t x_gyro_raw, y_gyro_raw, z_gyro_raw;
	int16_t x_acc_raw, y_acc_raw, z_acc_raw;
	
	float x_gyro_val, y_gyro_val, z_gyro_val;
	float x_acc_val, y_acc_val, z_acc_val;
	
	float x_acc_offset, y_acc_offset, z_acc_offset;
	float x_gyro_offset, y_gyro_offset, z_gyro_offset;
	
	float x_acc_angle, y_acc_angle, z_acc_angle;
	float x_gyro_angle_improved, y_gyro_angle_improved, z_gyro_angle_improved;
	float x_gyro_angle, y_gyro_angle, z_gyro_angle;
	float x_angle, y_angle, z_angle;
	
	Kalman *_x_kalman, *_y_kalman, *_z_kalman;
	
	//functions
	public:
	/* default constructor */
	IMU(uint8_t address, uint8_t pin_led);
	/* default destructor */
	~IMU();
	/* methods */
	uint8_t CheckConnection();
	void GetAndProcessData(bool kalamn_filtering, float gain = 0.7);
	void Calibrate(int nbr_samples);
	/* Getters */
	float GetSampleTime() { return dt; };
	float GetX() { return -y_angle; };// adaptation for the glove
	float GetY() { return -x_angle; };
	float GetZ() { return -z_angle; };
	float GetGyroX() { return -y_gyro_angle; };
	float GetGyroY() { return -x_gyro_angle; };
	float GetGyroZ() { return -z_gyro_angle; };
	float GetAccX() { return -y_acc_angle; };
	float GetAccY() { return -x_acc_angle; };
	float GetAccZ() { return z_acc_angle; };
	protected:
	private:
	/* reader */
	void GetRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
	/* low level interface */
	// read
	int ReadBytes(int start, uint8_t *buffer, int size);
	uint8_t ReadBytes(uint8_t regAddr, uint8_t length, uint8_t *data) ;
	uint8_t ReadByte(uint8_t regAddr, uint8_t *data) ;
	uint8_t ReadBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
	// write
	void WriteBytes(uint8_t regAddr, uint8_t length, uint8_t* data);
	void WriteByte(uint8_t regAddr, uint8_t data);
	void WriteBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
	void WriteBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
	
}; //IMU

#endif //__IMU_H__
