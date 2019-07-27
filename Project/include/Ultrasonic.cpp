/* 
* Ultrasonic.cpp
*
* Created: 09.07.2019 20:43:38
* Author: ussaema
*/


#include "Ultrasonic.h"

// default constructor
Ultrasonic::Ultrasonic(const uint8_t& pin_trig, const uint8_t& pin_echo): _pin_trig(pin_trig), _pin_echo(pin_echo)
{
	mcu.setPinFunction(pin_trig, OUTPUT);
	mcu.setPinFunction(pin_echo, INPUT);
	_en_kalman = false;
} //Ultrasonic

// default destructor
Ultrasonic::~Ultrasonic()
{
} //~Ultrasonic

uint16_t Ultrasonic::read()
{
	uint16_t data = read_raw();
	if (_en_kalman){
		_kalman->filter(data, 1);
		data = _kalman->filter(data, 1);
	}
	return data;
}

uint16_t Ultrasonic::read_raw()
{
	int duration, distance;
	mcu.setPinLevel(_pin_trig, HIGH);
	_delay_us(500);
	mcu.setPinLevel(_pin_trig, LOW);
	duration = capturePulse(_pin_echo, HIGH, 1000000L);
	distance = (uint16_t)((duration/2) / 29.1);
	return distance;
}

void Ultrasonic::enableKalman()
{
	_en_kalman = true;
	_kalman = new Kalman(1, 1, 1, 1, 1);
}

void Ultrasonic::disableKalman()
{
	_en_kalman = false;
	delete _kalman;
}

uint32_t Ultrasonic::countPulse(volatile uint8_t *port, uint8_t bit, uint8_t stateMask, uint32_t maxloops)
{
	uint32_t width = 0;
	// wait for any previous pulse to end
	while ((*port & bit) == stateMask)
	if (--maxloops == 0)
	return 0;
	
	// wait for the pulse to start
	while ((*port & bit) != stateMask)
	if (--maxloops == 0)
	return 0;
	
	// wait for the pulse to stop
	while ((*port & bit) == stateMask) {
		if (++width == maxloops)
		return 0;
	}
	return width;
}


uint32_t Ultrasonic::capturePulse(uint8_t pin, uint8_t state, uint32_t timeout)
{
	// cache the port and bit of the pin in order to speed up the
	// pulse width measuring loop and achieve finer resolution.  calling
	// getPinLevel() instead yields much coarser resolution.
	uint8_t bit = mcu.getPinMask(pin);
	uint8_t port = mcu.getPinPort(pin);
	uint8_t stateMask = (state ? bit : 0);

	// convert the timeout from microseconds to a number of times through
	// the initial loop; it takes approximately 16 clock cycles per iteration
	uint32_t maxloops = ( ((timeout)/16) * ( F_CPU / 1000000L ) );

	uint32_t width = countPulse(mcu.getPinInput(port), bit, stateMask, maxloops);

	// prevent clockCyclesToMicroseconds to return bogus values if countPulseASM timed out
	if (width)
	return ( (width * 16 + 16) / ( F_CPU / 1000000L ) );
	else
	return 0;
}