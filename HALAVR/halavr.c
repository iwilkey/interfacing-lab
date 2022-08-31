/*
	HALAVR is a hardware abstraction library created by Ian Wilkey as a utility for ECE 322 Interfacing Lab. It is intended to encapsulate all the common behaviors tools of the
	ATmega328p microprocessor in the context of ECE 322. This is the implementation file.

	@author Ian Wilkey (iwilkey)
	@date-created 8/30/2022
*/

#ifndef _HAL_AVR_C_
#define _HAL_AVR_C_

#include <avr/io.h>
#include "halavr.h"

uint8_t getStateOf(enum IOType type) {
	switch(type) {
		case PIB: return PINB; break;
		case PIC: return PINC; break;
		case PID: return PIND; break;
		case LEDS: return (PINB & 0b00111111); break;
		case BUTTONS: return (((~PIND) >> 4) & 0x0F); break;
		default: break;
	}
	return 0x00;
}

void setStateOf(enum IOType type, uint8_t value) {
	switch(type) {
		case PORB: PORTB &= value; break;
		case PORC: PORTC &= value; break;
		case PORD: PORTD &= value; break;
		case LEDS: PORTB = ((PORTB & 0b11000000) | (value & 0b00111111)); break;
		case BUTTONS: break;
		default: break;
	}
	return;
}

void delayMS(unsigned short int ms) {
	while(ms > 0) {
		volatile unsigned short int counter;
		for(counter = 0; counter < 886; counter++);
		ms--;
	}
	return;
}

void init(void) {
	DDRD = DDRD & 0x0F;
	PORTD = PORTD | 0x0F;
	DDRB = DDRB | 0b00111111;
	PORTB = PORTB & 0b11000000;
}

void dispose(void) {
	DDRD &= 0x0F;
	PORTD &= 0x0F;
	DDRB &= 0b11000000;
	PORTB &= 0b11000000;
}

int main(void) {
	return 0;
}

#endif // _HAL_AVR_C_
