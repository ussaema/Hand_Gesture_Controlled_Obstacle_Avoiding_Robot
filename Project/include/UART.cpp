/*
* UART.cpp
*
* Created: 09.07.2019 20:43:38
* Author: ussaema
*/

#include "UART.h"

/* interrupt called when the UART has received a byte */
ISR (USART_RX_vect)
{
	unsigned char tmphead;
	unsigned char lastRxError;
	// get frame error, data overrun and uart parity error bits
	lastRxError = UCSR0A & (_BV(FE0)|_BV(DOR0)|_BV(UPE0) );
	// get buffer index
	tmphead = ( UART_RxHead + 1) & ( UART_RX_BUFFER_SIZE - 1);
	if ( tmphead == UART_RxTail )
		// error: receive buffer overflow
		lastRxError = UART_BUFFER_OVERFLOW >> 8;
	else {
		// save the index
		UART_RxHead = tmphead;
		// store received data in buffer
		UART_RxBuf[tmphead] = UDR0;
	}
	UART_LastRxError |= lastRxError;
}

/* interrupt called when the UART is ready to transmit a byte */
ISR (USART_UDRE_vect)
{
	unsigned char tmptail;
	if ( UART_TxHead != UART_TxTail) {
		// calculate and store new buffer index
		tmptail = (UART_TxTail + 1) & ( UART_TX_BUFFER_SIZE - 1);
		UART_TxTail = tmptail;
		// get one byte from buffer and write it to UART
		UDR0 = UART_TxBuf[tmptail];  /* start transmission */
	} else {
		// tx buffer empty, disable UDRE interrupt
		UCSR0B &= ~_BV(UDRIE0);
	}
}

/* default constructor */
UART::UART()
{
}

/* default destructor */
UART::~UART()
{
} //~Ultrasonic

/* initialize */
void UART::Init(const uint16_t& baudrate)
{
	#if DEBUG == 1
	// compute baudrate in bps // double rate: (((((F_CPU) + 4ul * (baudrate)) / (8ul * (baudrate)) -1ul)) | 0x8000)
	_baudrate = (((F_CPU) + 8ul * (baudrate)) / (16ul * (baudrate)) -1ul);
	// stop all interrupts
	//cli();
	// set baudrate and set high speed transmission for high baudrates
	if ( _baudrate & 0x8000 ) UCSR0A = (1<<U2X0);
	UBRR0H = (unsigned char)((_baudrate>>8)&0x80) ;
	UBRR0L = (unsigned char) (_baudrate&0x00FF);
	// enable USART receiver and transmitter and receive complete interrupt
	UCSR0B = _BV(RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	// set frame format: asynchronous, 8data, no parity, 1stop bit
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	#endif
}

/* read */
uint16_t UART::Read()
{
	// no data available
	if (UART_RxHead == UART_RxTail ) return UART_NO_DATA;
	// calculate buffer index
	tmptail = (UART_RxTail + 1) & ( UART_RX_BUFFER_SIZE - 1);
	// get data from receive buffer
	data = UART_RxBuf[tmptail];
	lastRxError = UART_LastRxError;
	// store buffer index
	UART_RxTail = tmptail;
	// note that no errors occurred
	UART_LastRxError = 0;
	return (lastRxError << 8) + data;
}

/* write */
void UART::Write(char data)
{
	#if DEBUG == 1
	unsigned char tmphead;
	// get current head index
	tmphead  = (UART_TxHead + 1) & ( UART_TX_BUFFER_SIZE - 1);
	// wait for free space in buffer
	while ( tmphead == UART_TxTail );
	// send data to the transmitting buffer
	UART_TxBuf[tmphead] = data;
	// set the head index
	UART_TxHead = tmphead;
	// enable UDRE interrupt
	UCSR0B |= _BV(UDRIE0);
	#endif
}
void UART::Write(const char *data)
{
	#if DEBUG == 1
	while (*data)
	Write(*data++);
	#endif
}
void UART::Write(float data, int decimals)
{	
	#if DEBUG == 1
	char c[UART_FLOAT_BUFFER_SIZE];
	if (data <0) Write('-');
	if((int)data == 0) Write('0');
	ftoa(data, c, decimals);
	Write(c);
	#endif
}
void UART::Write(double data, int decimals)
{
	#if DEBUG == 1
	char c[UART_FLOAT_BUFFER_SIZE];
	if (data <0) Write('-');
	if((int)data == 0) Write('0');
	ftoa(data, c, decimals);
	Write(c);
	#endif
}

void UART::Write(uint8_t data)
{
	#if DEBUG == 1
	char c[UART_FLOAT_BUFFER_SIZE];
	itoa(data, c);
	Write(c);
	#endif
}

void UART::Write(uint16_t data)
{
	#if DEBUG == 1
	char c[UART_FLOAT_BUFFER_SIZE];
	itoa(data, c);
	Write(c);
	#endif
}

void UART::Write(uint32_t data)
{
	#if DEBUG == 1
	char c[UART_FLOAT_BUFFER_SIZE];
	itoa(data, c);
	Write(c);
	#endif
}


void UART::Write(int8_t data)
{
	#if DEBUG == 1
	if (data <0) Write('-');
	char c[UART_FLOAT_BUFFER_SIZE];
	itoa(data, c);
	Write(c);
	#endif
}

void UART::Write(int16_t data)
{
	#if DEBUG == 1
	if (data <0) Write('-');
	char c[UART_FLOAT_BUFFER_SIZE];
	itoa(data, c);
	Write(c);
	#endif
}

void UART::Write(int32_t data)
{
	#if DEBUG == 1
	if (data <0) Write('-');
	char c[UART_FLOAT_BUFFER_SIZE];
	itoa(data, c);
	Write(c);
	#endif
}

void UART::reverse(char *str, int len)
{
	int i=0, j=len-1, temp;
	while (i<j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}

int UART::intToStr(int x, char str[], int d)
{
	int i = 0;
	while (x)
	{
		str[i++] = (x%10) + '0';
		x = x/10;
	}
	// if number of digits required is more, then add 0s at the beginning
	while (i < d)
	str[i++] = '0';
	reverse(str, i);
	str[i] = '\0';
	return i;
}

void UART::ftoa(float n, char *res, int decimals)
{
	// check neg sign
	bool neg = n<0;
	n = neg ? -n : n;
	// extract integer part
	int ipart = (int)n;
	// extract floating part
	float fpart = n - (float)ipart;
	// convert integer part to string
	int i = intToStr(ipart, res, 0);
	// check for display option after point
	if (decimals != 0)
	{
		// add dot
		res[i] = '.';
		// get the value of fraction part up to given no
		fpart = fpart * pow(10, decimals);
		intToStr((int)fpart, res + i + 1, decimals);
	}
	
}


void UART::itoa(int32_t n, char *res)
{
	// check neg sign
	bool neg = n<0;
	n = neg ? -n : n;
	// extract integer part
	int ipart = (int)n;
	// convert integer part to string
	intToStr(ipart, res, 0);
}