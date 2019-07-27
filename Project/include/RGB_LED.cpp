/* 
* RGB_LED.cpp
*
* Created: 15.07.2019 15:26:35
* Author: ussaema
*/


#include "RGB_LED.h"

// default constructor
 RGB_LED::RGB_LED(const uint8_t pin_red, const uint8_t pin_green, const uint8_t pin_blue, const uint8_t intensity_on, const uint8_t intensity_off):
_pin_red(pin_red), _pin_green(pin_green), _pin_blue(pin_blue), _intensity_on(intensity_on), _intensity_off(intensity_off)
{
	Off();
}

// default destructor
RGB_LED::~RGB_LED()
{
} //~RGB_LED

void RGB_LED::White()
{
	mcu.setPinPWM(_pin_red, linear_mapping(1, 0, 1, _intensity_off, _intensity_on));
	mcu.setPinPWM(_pin_green, linear_mapping(1, 0, 1, _intensity_off, _intensity_on));
	mcu.setPinPWM(_pin_blue, linear_mapping(1, 0, 1, _intensity_off, _intensity_on));
}

void RGB_LED::Off()
{
	mcu.setPinPWM(_pin_red, linear_mapping(0, 0, 1, _intensity_off, _intensity_on));
	mcu.setPinPWM(_pin_green, linear_mapping(0, 0, 1, _intensity_off, _intensity_on));
	mcu.setPinPWM(_pin_blue, linear_mapping(0, 0, 1, _intensity_off, _intensity_on));
}

void RGB_LED::Set(float red_intensity, float green_intensity, float blue_intensity)
{
	mcu.setPinPWM(_pin_red, linear_mapping(red_intensity, 0, 1, _intensity_off, _intensity_on));
	mcu.setPinPWM(_pin_green, linear_mapping(green_intensity, 0, 1, _intensity_off, _intensity_on));
	mcu.setPinPWM(_pin_blue, linear_mapping(blue_intensity, 0, 1, _intensity_off, _intensity_on));
}
