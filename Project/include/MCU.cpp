/*
* MCU.cpp
*
* Created: 10.07.2019 21:00:55
* Author: ussaema
*/

#include "MCU.h"

/* constructor */
MCU::MCU(){
	
}

/* deconstructor */
MCU::~MCU(){
	
}

/* initialization */
void MCU::init()
{
	// set global interrupt enable
	sei();
	
	// --- timer 0
	// init
	TCCR0A = 0;
	TCCR0B = 0;
	// set timer in fast pwm mode
	setBit(TCCR0A, WGM01);
	setBit(TCCR0A, WGM00);
	// set timer prescale factor to 64
	setBit(TCCR0B, CS01);
	setBit(TCCR0B, CS00);
	// enable timer overflow interrupt
	setBit(TIMSK0, TOIE0);
	
	// --- timer 1
	// init
	TCCR1A = 0;
	TCCR1B = 0;
	// set timer prescale factor to 64
	setBit(TCCR1B, CS11); setBit(TCCR1B, CS10);
	// set timer in 16bit phase correct pwm mode
	setBit(TCCR1A, WGM10);
	
	// --- timer 2
	TCCR2A = 0;
	TCCR2B = 0;
	// set timer prescale factor to 64
	setBit(TCCR2B, CS22);
	// set timer in 8bit phase correct pwm mode
	setBit(TCCR2A, WGM20);
	
	// --- adc
	// set adc prescaler to 128
	setBit(ADCSRA, ADPS0);
	setBit(ADCSRA, ADPS1);
	setBit(ADCSRA, ADPS2);
	// enable adc conversion
	setBit(ADCSRA, ADEN);
}

/* IO functions */
// mapping
volatile uint8_t* MCU::getPinFunction(uint8_t port)
{
	if (port == PD) return &DDRD;
	if (port == PB) return &DDRB;
	if (port == PC) return &DDRC;
	return NO_FUNCTION;
}
volatile uint8_t* MCU::getPinOutput(uint8_t port)
{
	if (port == PD) return &PORTD;
	if (port == PB) return &PORTB;
	if (port == PC) return &PORTC;
	return NO_OUTPUT;
}
volatile uint8_t* MCU::getPinInput(uint8_t port)
{
	if (port == PD) return &PIND;
	if (port == PB) return &PINB;
	if (port == PC) return &PINC;
	return NO_INPUT;
}
volatile uint8_t MCU::getPinPort(uint8_t pin)
{
	if (pin >= 0 && pin <= 7) return PD;
	if (pin >= 8 && pin <= 13) return PB;
	if (pin >= 14 && pin <= 19) return PC;
	return NO_PORT;
}
volatile uint8_t MCU::getPinMask(uint8_t pin)
{
	if (pin >= 0 && pin <= 7) return _BV(pin);
	if (pin >= 8 && pin <= 13) return _BV(pin-8);
	if (pin >= 14 && pin <= 19) return _BV(pin-14);
	return NO_MASK;
}
volatile uint8_t MCU::getPinTimer(uint8_t pin)
{
	if (pin == 3) return TIMER2B;
	if (pin == 5) return TIMER0B;
	if (pin == 6) return TIMER0A;
	if (pin == 9) return TIMER1A;
	if (pin == 10) return TIMER1B;
	if (pin == 11) return TIMER2A;
	return NO_TIMER;
}

// setters
void MCU::setPinFunction(uint8_t pin, uint8_t function)
{
	// get the mapping
	uint8_t bit = getPinMask(pin);
	uint8_t port = getPinPort(pin); if (port == NO_PIN) return;
	volatile uint8_t *fct, *out;
	fct = getPinFunction(port);
	out = getPinOutput(port);

	// store status register and stop interrupts
	uint8_t oldSREG = SREG;
	cli();
	if (function == INPUT) {
		// clear function and output
		*fct &= ~bit;
		*out &= ~bit;
	} else if (function == OUTPUT) {
		// set function bit
		*fct |= bit;
	}
	// reset the status register
	SREG = oldSREG;
}



void MCU::setPinLevel(uint8_t pin, uint8_t level)
{
	// get the mapping
	uint8_t bit = getPinMask(pin);
	uint8_t port = getPinPort(pin); if (port == NO_PIN) return;
	uint8_t timer = getPinTimer(pin); if (timer != NO_TIMER) setTimerOff(timer); // be sure that timer associated to this pin is off
	volatile uint8_t *out;
	out = (volatile uint8_t *)( getPinOutput(port));
	
	// store status register and stop interrupts
	uint8_t oldSREG = SREG;
	cli();
	if (level == LOW) {
		// clear output pin
		*out &= ~bit;
	} else {
		// set output pin
		*out |= bit;
	}
	// reset the status register
	SREG = oldSREG;
}

void MCU::setPinPWM(uint8_t pin, uint16_t val)
{
	setPinFunction(pin, OUTPUT); // make sure output function is turned on
	// if extreme values are given, setting the level with duty cycle 0% or 100% will not require a timer to be on
	if (val == 0) setPinLevel(pin, LOW); 
	else if (val == 255) setPinLevel(pin, HIGH); 
	else
	{
		switch(getPinTimer(pin))
		{
			// timer 0 channel A
			case TIMER0A:
			setBit(TCCR0A, COM0A1); // clear OCR0A on compare match
			OCR0A = val; // set pwm duty
			break;
			// timer 0 channel B
			case TIMER0B:
			setBit(TCCR0A, COM0B1); // clear OCR0B on compare match
			OCR0B = val; // set pwm duty
			break;
			// timer 1 channel A
			case TIMER1A:
			setBit(TCCR1A, COM1A1); // clear OCR1A on compare match
			OCR1A = val; // set pwm duty
			break;
			// timer 1 channel B
			case TIMER1B:
			setBit(TCCR1A, COM1B1); // clear OCR1B on compare match
			OCR1B = val; // set pwm duty
			break;
			// timer 2 channel A
			case TIMER2A:
			setBit(TCCR2A, COM2A1); // clear OCR2A on compare match
			OCR2A = val; // set pwm duty
			break;
			// timer 2 channel B
			case TIMER2B:
			setBit(TCCR2A, COM2B1); // clear OCR2B on compare match
			OCR2B = val; // set pwm duty
			break;
			// if not a timer, set level with threshold 128
			case NO_TIMER:
			default:
			if (val < 128) {
				setPinLevel(pin, LOW);
				} else {
				setPinLevel(pin, HIGH);
			}
		}
	}
}

void MCU::setTimerOff(uint8_t timer)
{
	switch (timer)
	{
		// timer 0
		case  TIMER0A:  clearBit(TCCR0A, COM0A1);    break;
		case  TIMER0B:  clearBit(TCCR0A, COM0B1);    break;
		// timer 1
		case TIMER1A:   clearBit(TCCR1A, COM1A1);    break;
		case TIMER1B:   clearBit(TCCR1A, COM1B1);    break;
		// timer 2
		case  TIMER2A:  clearBit(TCCR2A, COM2A1);    break;
		case  TIMER2B:  clearBit(TCCR2A, COM2B1);    break;
	}
}

// getters
uint8_t MCU::getPinLevel(uint8_t pin)
{
	uint8_t bit = getPinMask(pin);
	uint8_t port = getPinPort(pin); if (port == NO_PIN) return LOW;
	uint8_t timer = getPinTimer(pin); if (timer != NO_TIMER) setTimerOff(timer); // be sure that timer associated to this pin is off
	return (*getPinInput(port) & bit) ? HIGH : LOW ;
}

uint16_t MCU::getPinADC(uint8_t pin)
{
	uint8_t low, high;
	// get the reference pin to the associated port
	if (pin >= 14) pin -= 14;
	// set the analog reference and ADLAR (to 0) and select the channel
	ADMUX = (1 << 6) | (pin & 0x07);
	// start conversion
	setBit(ADCSRA, ADSC);
	// wait until the conversion is done
	while (readBit(ADCSRA, ADSC));
	// read the ADC values
	low  = ADCL;
	high = ADCH;
	// combine to 16 bit
	return (high << 8) | low;
}

/* timing */
volatile unsigned long time_counter = 0;
volatile unsigned long time_ms = 0;
static unsigned char time_fraction = 0;

ISR(TIMER0_OVF_vect)
{
	// read the registers from the memory
	unsigned long m = time_ms;
	unsigned char f = time_fraction;
	// compute the time in ms and fraction
	m += (((64 * 256) / (F_CPU / 1000000L)) / 1000);
	f += ((((64 * 256) / ( F_CPU / 1000000L ) ) % 1000) >> 3);
	if (f >= (1000 >> 3)) {
		f -= (1000 >> 3);
		m += 1;
	}
	// write to the registers in memory
	time_fraction = f;
	time_ms = m;
	time_counter++;
}

unsigned long MCU::getTime()
{
	unsigned long m;
	// store status register and stop interrupts to allow a coherent reading of the register
	uint8_t oldSREG = SREG;
	cli();
	// store the time
	m = time_ms;
	// reset the status register
	SREG = oldSREG;
	return m;
}