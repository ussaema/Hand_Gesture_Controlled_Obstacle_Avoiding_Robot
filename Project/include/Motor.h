/* 
* Motor.h
*
* Created: 10.07.2019 20:02:50
* Author: ussaema
*/


#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "MCU.h"

class Motor
{
//variables
public:
protected:
private:
	uint8_t _pin_ctrl;
	uint8_t _pin_forward;
	uint8_t _pin_backward;
	uint8_t _speed;
//functions
public:
	Motor(uint8_t pin_ctrl, uint8_t pin_forward, uint8_t pin_backward);
	void setSpeed(uint8_t speed);
	void turnForward();
	void turnBackward();
	~Motor();
protected:
private:

}; //Motor

#endif //__MOTOR_H__
