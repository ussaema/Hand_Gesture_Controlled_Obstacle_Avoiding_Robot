/* 
* Infrared.cpp
*
* Created: 09.07.2019 20:27:23
* Author: ussaema
*/


#include "Infrared.h"

// default constructor
Infrared::Infrared(const uint8_t& pin): _pin(pin)
{
	mcu.setPinFunction(pin, INPUT);
	_en_kalman = false;
} //Infrared

// default destructor
Infrared::~Infrared()
{
} //~Infrared

uint16_t Infrared::read()
{
	uint16_t data = read_raw();
	if (_en_kalman){
		_kalman->filter(data, 0);
		data = _kalman->filter(data, 0);
	}
	return data;
}

uint16_t Infrared::read_raw()
{
	return (uint8_t)linear_mapping(mcu.getPinADC(_pin), 500, 900, 0, 255);
	
}

void Infrared::enableKalman()
{
	_en_kalman = true;
	_kalman = new Kalman(1, 1, 1, 1, 1);
}

void Infrared::disableKalman()
{
	_en_kalman = false;
	delete _kalman;
}
