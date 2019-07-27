/* 
* Infrared.h
*
* Created: 09.07.2019 20:27:23
* Author: ussaema
*/


#ifndef __INFRARED_H__
#define __INFRARED_H__
#include "MCU.h"
#include "Utility.h"
#include "Kalman.h"
#include "UART.h"

class Infrared
{
//variables
public:
protected:
private:
	uint8_t _pin;
	Kalman *_kalman; // left infrared filter
	bool _en_kalman;
//functions
public:
	Infrared(const uint8_t& pin);
	~Infrared();
	uint16_t read();
	uint16_t read_raw();
	void enableKalman();
	void disableKalman();
protected:
private:

}; //Infrared

#endif //__INFRARED_H__
