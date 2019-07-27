/* 
* VibrationMotor.cpp
*
* Created: 23.07.2019 15:46:58
* Author: ussaema
*/


#include "VibrationMotor.h"

// default constructor
VibrationMotor::VibrationMotor(uint8_t pin): _pin(pin)
{
} //VIbrationMotor

// default destructor
VibrationMotor::~VibrationMotor()
{
} //~VIbrationMotor

void VibrationMotor::Off()
{
	mcu.setPinLevel(_pin, LOW);
}

void VibrationMotor::On()
 {
	 mcu.setPinLevel(_pin, HIGH);
 }

void VibrationMotor::Set(float intensity)
 {
	 mcu.setPinPWM(_pin, intensity);
 }
