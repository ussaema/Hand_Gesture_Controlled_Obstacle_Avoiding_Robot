/*
* IMU.cpp
*
* Created: 07.07.2019 10:23:03
* Author: ussaema
*/

#include "IMU.h"

/* default constructor */
IMU::IMU(uint8_t address, uint8_t pin_led): _address(address), _pin_led(pin_led)
{
	// wait for signals to stabilize
	_delay_ms(100);
	// set sleep disabled
	WriteBit(IMU_PWR_MGMT_1, IMU_PWR1_SLEEP_BIT, 0);
	// wait for the previous command to be executed
	_delay_ms(10);
	// set clock source to the the gyroscope clock
	WriteBits(IMU_PWR_MGMT_1, IMU_PWR1_CLKSEL_BIT, IMU_PWR1_CLKSEL_LENGTH, IMU_CLOCK_PLL_XGYRO);
	//set DLPF bandwidth to 42Hz
	WriteBits(IMU_CONFIG, IMU_CFG_DLPF_CFG_BIT, IMU_CFG_DLPF_CFG_LENGTH, IMU_DLPF_BW_42);
	// set sample rate
	WriteByte(IMU_SMPLRT_DIV, 4); //1khz / (1 + 4) = 200Hz
	//set gyroscope range
	WriteBits(IMU_GYRO_CONFIG, IMU_GCONFIG_FS_SEL_BIT, IMU_GCONFIG_FS_SEL_LENGTH, IMU_GYRO_FS_2000);
	//set accelerator range
	WriteBits(IMU_ACCEL_CONFIG, IMU_ACONFIG_AFS_SEL_BIT, IMU_ACONFIG_AFS_SEL_LENGTH, IMU_GYRO_FS_2000);
	
	// initialize the instances @ t=0 (integral computation)
	t0 = 0;
	t1 = 0;
	x_angle = 0; x_gyro_angle = 0;
	y_angle = 0; y_gyro_angle = 0;
	z_angle = 0; z_gyro_angle = 0;
	
	// set the debug led
	mcu.setPinFunction(pin_led, OUTPUT);
	
	// open I2C communication interface
	_x_kalman = new Kalman(1, 1, 1, 1, 1);
	_y_kalman = new Kalman(1, 1, 1, 1, 1);
	_z_kalman = new Kalman(1, 1, 1, 1, 1);
	
	// calibration
	Calibrate(100);
	
} //IMU

/* default destructor */
IMU::~IMU()
{
} //~IMU

/* methods */
uint8_t IMU::CheckConnection() {
	ReadBits(IMU_WHO_AM_I, IMU_WHO_AM_I_BIT, IMU_WHO_AM_I_LENGTH, (uint8_t *)buffer);
	if(buffer[0] == 0x34) return 1;
	else return 0;
}
void IMU::GetAndProcessData(bool kalman_filtering, float gain)
{
	mcu.setPinLevel(_pin_led, HIGH);
	GetRawData(&x_acc_raw, &y_acc_raw, &z_acc_raw, &x_gyro_raw, &y_gyro_raw, &z_gyro_raw);
	
	// store time used for the integral
	t1 = mcu.getTime();
	
	// normalize the raw data
	x_gyro_val = (x_gyro_raw - x_gyro_offset)/GYROGAIN;
	y_gyro_val = (y_gyro_raw - y_gyro_offset)/GYROGAIN;
	z_gyro_val = (z_gyro_raw - z_gyro_offset)/GYROGAIN;
	
	x_acc_val = (x_acc_raw ) /ACCGAIN;
	y_acc_val = (y_acc_raw ) /ACCGAIN;
	z_acc_val = (z_acc_raw )/ACCGAIN;
		
	// get the angle information from the accelerometer sensor data
	x_acc_angle = atan(y_acc_val/sqrt(pow(x_acc_val,2) + pow(z_acc_val,2)))*RADIANS_TO_DEGREES;
	y_acc_angle = atan(-1*x_acc_val/sqrt(pow(y_acc_val,2) + pow(z_acc_val,2)))*RADIANS_TO_DEGREES;
	z_acc_angle = 0;
	
	// get the angle information from the gyrometer sensor data
	dt = (t1-t0) / 1000.; dt = dt>0.1 ? 0.05: dt;
	x_gyro_angle_improved = x_gyro_val*dt + x_angle; if (x_gyro_angle_improved < -360 || x_gyro_angle_improved > 360)  x_gyro_angle_improved = x_gyro_val*dt;
	y_gyro_angle_improved = y_gyro_val*dt + y_angle; if (y_gyro_angle_improved < -360 || y_gyro_angle_improved > 360)  y_gyro_angle_improved = y_gyro_val*dt;
	z_gyro_angle_improved = z_gyro_val*dt + z_angle; if (z_gyro_angle_improved < -360 || z_gyro_angle_improved > 360)  z_gyro_angle_improved = z_gyro_val*dt;
	
	x_gyro_angle = x_gyro_val*dt + x_gyro_angle; if (x_gyro_angle < -360 || x_gyro_angle > 360)  x_gyro_angle = x_gyro_val*dt;
	y_gyro_angle = y_gyro_val*dt + y_gyro_angle; if (y_gyro_angle < -360 || y_gyro_angle > 360)  y_gyro_angle = y_gyro_val*dt;
	z_gyro_angle = z_gyro_val*dt + z_gyro_angle; if (z_gyro_angle < -360 || z_gyro_angle > 360)  z_gyro_angle = z_gyro_val*dt;
	
	// sensor fusion: get the filtered angle information by performing the complementary filtering
	x_angle = gain*x_gyro_angle_improved + (1.0 - gain)*x_acc_angle;
	y_angle = gain*y_gyro_angle_improved + (1.0 - gain)*y_acc_angle;
	z_angle = z_gyro_angle_improved;
	t0 = t1;
	
	if (kalman_filtering){
		_x_kalman->filter(x_angle, 0);
		x_angle = _x_kalman->filter(x_angle, 0);
		
		_y_kalman->filter(y_angle, 0);
		y_angle = _y_kalman->filter(y_angle, 0);
		
		_z_kalman->filter(z_angle, 0);
		z_angle = _z_kalman->filter(z_angle, 0);
	}
	
	mcu.setPinLevel(_pin_led, LOW);
	#if DEMO == 1
	uart.Write("DEL:");              //Delta T
	uart.Write(GetSampleTime());
	uart.Write("#ACC:");              //Accelerometer angle
	uart.Write(GetAccX(), 2);
	uart.Write(",");
	uart.Write(GetAccY(), 2);
	uart.Write(",");
	uart.Write(GetAccZ(), 2);
	uart.Write("#GYR:");
	uart.Write(GetGyroX(), 2);        //Gyroscope angle
	uart.Write(",");
	uart.Write(GetGyroY(), 2);
	uart.Write(",");
	uart.Write(GetGyroZ(), 2);
	uart.Write("#FIL:");             //Filtered angle
	uart.Write(GetX(), 2);
	uart.Write(",");
	uart.Write(GetY(), 2);
	uart.Write(",");
	uart.Write(GetZ(), 2);
	uart.Write("\r\n");
	#endif
}

void IMU::Calibrate(int nbr_samples) {
	
	// initilize the offsets
	x_acc_offset = 0; y_acc_offset = 0; z_acc_offset = 0;
	x_gyro_offset = 0; y_gyro_offset = 0; z_gyro_offset = 0;
	
	//uart.Write("Starting Calibration/r/n");

	int16_t x_gyro_raw, y_gyro_raw, z_gyro_raw;
	int16_t x_acc_raw, y_acc_raw, z_acc_raw;
	// discard the first values since they are most likely to be noisy
	GetRawData(&x_acc_raw, &y_acc_raw, &z_acc_raw, &x_gyro_raw, &y_gyro_raw, &z_gyro_raw);
	
	// compute the mean of the recoded data (the center of the data)
	
	for (int i = 0; i < nbr_samples; i++) {
		GetRawData(&x_acc_raw, &y_acc_raw, &z_acc_raw, &x_gyro_raw, &y_gyro_raw, &z_gyro_raw);
		x_acc_offset += x_acc_raw;
		y_acc_offset += y_acc_raw;
		z_acc_offset += z_acc_raw;
		x_gyro_offset += x_gyro_raw;
		y_gyro_offset += y_gyro_raw;
		z_gyro_offset += z_gyro_raw;
		//delay(100);
	}
	x_acc_offset /= nbr_samples;
	y_acc_offset /= nbr_samples;
	z_acc_offset /= nbr_samples;
	x_gyro_offset /= nbr_samples;
	y_gyro_offset /= nbr_samples;
	z_gyro_offset /= nbr_samples;
	
	//uart.Write("Finishing Calibration/r/n");
}

/* reader */
void IMU::GetRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
	ReadBytes(ACCGYRO_ADDR, 14, (uint8_t *)buffer);

	*ax = (((int16_t)buffer[0]) << 8) | buffer[1];
	*ay = (((int16_t)buffer[2]) << 8) | buffer[3];
	*az = (((int16_t)buffer[4]) << 8) | buffer[5];
	*gx = (((int16_t)buffer[8]) << 8) | buffer[9];
	*gy = (((int16_t)buffer[10]) << 8) | buffer[11];
	*gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

/* low level interface */
// read
uint8_t IMU::ReadBytes(uint8_t regAddr, uint8_t length, uint8_t *data) {
	uint8_t i = 0;
	int8_t count = 0;
	if(length > 0) {
		//request register
		i2c_start((_address <<1) | I2C_WRITE);
		i2c_write(regAddr);
		_delay_us(10);
		//read data
		i2c_start((_address <<1) | I2C_READ);
		for(i=0; i<length; i++) {
			count++;
			if(i==length-1)
				data[i] = i2c_readNak();
			else
				data[i] = i2c_readAck();
		}
		i2c_stop();
	}
	return count;
}

uint8_t IMU::ReadByte(uint8_t regAddr, uint8_t *data) {
    return ReadBytes(regAddr, 1, data);
}

uint8_t IMU::ReadBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    int8_t count = 0;
    if(length > 0) {
		uint8_t b;
		if ((count = ReadByte(regAddr, &b)) != 0) {
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			b &= mask;
			b >>= (bitStart - length + 1);
			*data = b;
		}
    }
    return count;
}

// write
void IMU::WriteBytes(uint8_t regAddr, uint8_t length, uint8_t* data) {
	if(length > 0) {
		i2c_start((_address <<1) | I2C_WRITE);
		i2c_write(regAddr); //reg
		for (uint8_t i = 0; i < length; i++) {
			i2c_write((uint8_t) data[i]);
		}
		i2c_stop();
	}
}

void IMU::WriteByte(uint8_t regAddr, uint8_t data) {
    return WriteBytes(regAddr, 1, &data);
}

void IMU::WriteBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
	if(length > 0) {
		uint8_t b = 0;
		if (ReadByte(regAddr, &b) != 0) { //get current data
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			data <<= (bitStart - length + 1); // shift data into correct position
			data &= mask; // zero all non-important bits in data
			b &= ~(mask); // zero all important bits in existing byte
			b |= data; // combine data with existing byte
			WriteByte(regAddr, b);
		}
	}
}

void IMU::WriteBit(uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    ReadByte(regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    WriteByte(regAddr, b);
}