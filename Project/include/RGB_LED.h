/* 
* RGB_LED.h
*
* Created: 15.07.2019 15:26:35
* Author: ussaema
*/


#ifndef __RGB_LED_H__
#define __RGB_LED_H__
#include "Utility.h"
#include "MCU.h"

class RGB_LED
{
//variables
public:
protected:
private:
	uint8_t _pin_red;
	uint8_t _pin_green;
	uint8_t _pin_blue;
	uint8_t _intensity_on;
	uint8_t _intensity_off;
//functions
public:
	RGB_LED(const uint8_t pin_red, const uint8_t pin_green, const uint8_t pin_blue, const uint8_t intensity_on, const uint8_t intensity_off);
	~RGB_LED();
	void White();
	void Off();
	void Set(float red_intensity, float green_intensity, float blue_intensity);
protected:
private:

}; //RGB_LED

#endif //__RGB_LED_H__
