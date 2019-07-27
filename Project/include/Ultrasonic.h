/* 
* Ultrasonic.h
*
* Created: 09.07.2019 20:43:38
* Author: ussaema
*/


#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__
#include "MCU.h"
#include "Kalman.h"

class Ultrasonic
{
//variables
public:
protected:
private:
	uint8_t _pin_trig;
	uint8_t _pin_echo;
	Kalman *_kalman; // left infrared filter
	bool _en_kalman;
//functions
public:
	Ultrasonic(const uint8_t& pin_echo, const uint8_t& pin_trig);
	~Ultrasonic();
	uint16_t read();
	uint16_t read_raw();
	void enableKalman();
	void disableKalman();
protected:
private:
	uint32_t capturePulse(uint8_t pin, uint8_t state, uint32_t timeout = 1000000L);
	uint32_t countPulse(volatile uint8_t *port, uint8_t bit, uint8_t stateMask, uint32_t maxloops);

}; //Ultrasonic

#endif //__ULTRASONIC_H__
