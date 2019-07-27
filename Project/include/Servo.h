/*
* Servo.h
*
* Created: 14.07.2019 12:21:30
* Author: ussaema
*/

#ifndef Servo_h
#define Servo_h

#include <inttypes.h>
#include "Utility.h"
#include "MCU.h"

#define MIN_PULSE_WIDTH 600 // the shortest pulse
#define MAX_PULSE_WIDTH 2400 // the longest pulse
#define DEFAULT_PULSE_WIDTH 1500 // default pulse
#define REFRESH_INTERVAL 20000 // minimum time to refresh servos in microseconds

class Servo
{
	//variables
	public:
	protected:
	private:
	uint8_t _pin;
	//functions
	public:
	Servo(uint8_t pin);
	~Servo();
	void init();
	void write(int value);
	protected:
	private:
};

#if DEVICE == 1
static Servo servo(PIN_R_SRV); 
#endif
#endif
