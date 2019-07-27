/*
* MCU.h
*
* Created: 10.07.2019 21:00:55
* Author: ussaema
*/

#ifndef __MCU_H__
#define __MCU_H__

//--------------------------------------------------------------
// Configuration, clock, pin, rates definition
//--------------------------------------------------------------
/* general config */
#define DEVICE 1// 0: Smart Hand, 1: Obstacle Avoiding Robot
#define DEBUG 1 // en/disable uart debugger
#define DEMO 1
#define UART_BAUD_RATE 19200
#define F_CPU 16000000UL

/* PINs Obstacle Avoiding Robot (OAR) */
#if DEVICE == 1
#define MAX_VELOCITY 255
// infrared sensors
#define PIN_R_IR_LEFT A2
#define PIN_R_IR_RIGHT A1
#define PIN_R_IR_BACK A0
// ultrasonic sensor
#define PIN_R_US_TRIG A5
#define PIN_R_US_ECHO A4
// motors
#define PIN_R_M_CTRL_LEFT 6
#define PIN_R_M_CTRL_RIGHT 5
#define PIN_R_M_FORWARD_LEFT 7
#define PIN_R_M_FORWARD_RIGHT 2
#define PIN_R_M_BACKWARD_LEFT 8
#define PIN_R_M_BACKWARD_RIGHT 4
// servo
#define PIN_R_SRV 3
// radio sender/receiver
#define PIN_R_RF_CE 9
#define PIN_R_RF_CS 10


/* PINs Smart Hand (SH) */
#elif DEVICE == 0
// --- leds
#define PIN_SH_LED_R 7
#define PIN_SH_LED_G 6
#define PIN_SH_LED_B 5
#define LED_INTENSITY_ON 0
#define LED_INTENSITY_OFF 255
#define PIN_SH_LED1 A1
#define PIN_SH_LED2 8
// --- vibration motor
#define PIN_SH_VM 3
// --- IMU
#define ADDR_SH_IMU 0x68
#define PIN_SH_BTN A0
// radio sender/receiver
#define PIN_SH_RF_CE 9
#define PIN_SH_RF_CS 10

#endif

//--------------------------------------------------------------
// Libraries: C, AVR
//--------------------------------------------------------------
/* C libraries */
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
/* AVR libraries */
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//--------------------------------------------------------------
// Micro controller (Atmega328p): standard operations
// (Register R/W, interrupts, PWMs, ...)
//--------------------------------------------------------------

/* clear and set bit in a reg */
#define clearBit(reg, bit) (reg &= ~_BV(bit))
#define setBit(reg, bit) (reg |= _BV(bit))
#define readBit(reg , bit) (reg & _BV(bit))

/* PIN reg */
#define NO_INPUT (uint8_t*) 0xFF
/* PORT reg */
#define NO_OUTPUT (uint8_t*) 0xFF
/* _BV reg */
#define NO_MASK 0xFF
/* general GPIO */
#define NO_PIN 0xFF

/* pin functions (DDR reg) */
#define NO_FUNCTION (uint8_t*) 0xFF
#define INPUT 0
#define OUTPUT 1

/* pin levels */
#define NO_LEVEL 0xFF
#define LOW  0
#define HIGH 1

/* pin ports */
#define NO_PORT 0xFF
#define PD 0
#define PB 1
#define PC 2

/* pin timers */
#define NO_TIMER 0xFF
#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER2A 5 // 7
#define TIMER2B 6 // 8

/* pin ADC */
#define NO_ADC 0xFF
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21

/* pin SPI */
#define SS 10
#define MOSI 11
#define MISO 12
#define SCK 13

/* pin I2C */
#define SDA 18
#define SCL 19

class MCU
{
	//variables
	public:
		int16_t temp ;
	protected:
	private:
	//functions
	public:
	MCU();
	~MCU();
	/* initialization */
	void init();
	/* IO functions */
	// mapping
	volatile uint8_t* getPinFunction(uint8_t port);
	volatile uint8_t* getPinOutput(uint8_t port);
	volatile uint8_t* getPinInput(uint8_t port);
	volatile uint8_t getPinPort(uint8_t pin);
	volatile uint8_t getPinMask(uint8_t pin);
	volatile uint8_t getPinTimer(uint8_t pin);
	// setters
	void setPinFunction(uint8_t pin, uint8_t function);
	void setPinLevel(uint8_t pin, uint8_t level);
	void setPinPWM(uint8_t pin, uint16_t value);
	void setTimerOff(uint8_t timer);
	// getters
	uint8_t getPinLevel(uint8_t pin);
	uint16_t getPinADC(uint8_t pin);
	/* timing */
	unsigned long getTime(void); // escaped time in ms
	protected:
	private:
	
}; //MCU

static MCU mcu;

#endif //__MCU_H__
