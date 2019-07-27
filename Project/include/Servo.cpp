/*
* Servo.cpp
*
* Created: 14.07.2019 12:21:30
* Author: ussaema
*/

#include <avr/interrupt.h>

#include "Servo.h"
#define LOW 0
#define HIGH 1
volatile unsigned int value_reg;
volatile unsigned int signal_pin;

/****************** end of static functions ******************************/

Servo::Servo(uint8_t pin): _pin(pin)
{
	// store the pin number to memory
	signal_pin = pin;
	
}


 Servo::~Servo()
{

}

void Servo::init(){
	// initialize timer
	TCCR1A = 0;
	// set prescaler to 8
	TCCR1B = _BV(CS11);
	// clear the timer count
	TCNT1 = 0;
	// clear the corresponding interrupt
	TIFR1 |= _BV(OCF1A);
	// enable output compare interrupt
	TIMSK1 |=  _BV(OCIE1A) ;
}

#if DEVICE == 1
volatile uint8_t srv_pos_dir = 0;
volatile uint8_t pos = 0;

ISR (TIMER1_COMPA_vect)
{
	// generate the pulse
	mcu.setPinLevel(signal_pin,LOW);
	mcu.setPinLevel(signal_pin,HIGH);
	// set the duty cycle
	OCR1A = TCNT1 + value_reg; // TCNT1 is reset equal 0
	
	if (OCR1A % 10 == 0){
		if (srv_pos_dir) pos ++;
		else pos --;
		if (pos == 20) srv_pos_dir = 0;
		if (pos == 0) srv_pos_dir = 1;
		servo.write(90+pos);
	}

}
#endif
void Servo::write(int value)
{
	// be sure that the angle is within the range
	if(value < 0) value = 0;
	if(value > 180) value = 180;
	value = linear_mapping(value, 0, 180, MIN_PULSE_WIDTH,  MAX_PULSE_WIDTH);
	// calculate the register value
	value = (( F_CPU / 1000000L)* value) / 8;
	// store status register and stop interrupts
	uint8_t oldSREG = SREG;
	cli();
	// set the register value
	value_reg = value;
	// reset the status register
	SREG = oldSREG;
}