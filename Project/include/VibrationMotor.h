/* 
* VibrationMotor.h
*
* Created: 23.07.2019 15:46:58
* Author: ussaema
*/


#ifndef __VIBRATIONMOTOR_H__
#define __VIBRATIONMOTOR_H__

#include "MCU.h"

class VibrationMotor
{
//variables
public:
protected:
private:
	uint8_t _pin;
//functions
public:
	VibrationMotor(uint8_t pin);
	~VibrationMotor();
	void Off();
	void On();
	void Set(float intensity);
protected:
private:

}; //VibrationMotor

#endif //__VIBRATIONMOTOR_H__
