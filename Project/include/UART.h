/*
* UART.h
*
* Created: 09.07.2019 20:43:38
* Author: ussaema
*/

#ifndef __UART_H__
#define __UART_H__

#include "MCU.h"

#include <avr/interrupt.h>
#include <util/delay.h>

#define UART_BUFFER_OVERFLOW  0x0200
#define UART_NO_DATA          0x0100

#define UART_RX_BUFFER_SIZE 32
#define UART_TX_BUFFER_SIZE 32
#define UART_FLOAT_BUFFER_SIZE 64

static volatile unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead = 0;
static volatile unsigned char UART_TxTail = 0;
static volatile unsigned char UART_RxHead = 0;
static volatile unsigned char UART_RxTail = 0;
static volatile unsigned char UART_LastRxError;

class UART
{
	//variables
	public:
	protected:
	private:
	uint16_t _baudrate;
	unsigned char tmptail;
	unsigned char data;
	unsigned char lastRxError;

	//functions
	public:
	UART();
	~UART();
	void Init(const uint16_t& baudrate);
	uint16_t Read();
	void Write(char data);
	void Write(const char *data);
	void Write(float data, int decimals = 5);
	void Write(double data, int decimals = 5);
	void Write(uint8_t data);
	void Write(uint16_t data);
	void Write(uint32_t data);
	void Write(int8_t data);
	void Write(int16_t data);
	void Write(int32_t data);
	protected:
	private:
	void reverse(char *str, int len);
	int intToStr(int x, char str[], int d);
	void ftoa(float n, char *res, int decimals);
	void itoa(int32_t n, char *res);
}; //UART

static UART uart;

#endif //__UART_H__
