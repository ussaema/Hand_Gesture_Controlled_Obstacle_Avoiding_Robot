/* 
* Motor.cpp
*
* Created: 10.07.2019 20:02:50
* Author: ussaema
*/


#include "Motor.h"

// default constructor
 Motor::Motor(uint8_t pin_ctrl, uint8_t pin_forward, uint8_t pin_backward): _pin_ctrl(pin_ctrl), _pin_forward(pin_forward), _pin_backward(pin_backward)
{
	mcu.setPinFunction(pin_ctrl, OUTPUT);
	mcu.setPinFunction(pin_forward, OUTPUT);
	mcu.setPinFunction(pin_backward, OUTPUT);
}

void Motor::setSpeed(uint8_t speed)
{
	_speed = speed;
	mcu.setPinPWM(_pin_ctrl, _speed);
}

void Motor::turnForward()
{
	mcu.setPinLevel(_pin_backward, LOW);
	mcu.setPinLevel(_pin_forward, HIGH);
}

void Motor::turnBackward()
{
	mcu.setPinLevel(_pin_forward, LOW);
	mcu.setPinLevel(_pin_backward, HIGH);
}

// default destructor
Motor::~Motor()
{
} //~Motor
