/*
	HALAVR is a hardware abstraction library created by Ian Wilkey as a utility for ECE 322 Interfacing Lab. It is intended to encapsulate all the common behaviors tools of the
	ATmega328p microprocessor in the context of ECE 322. This is the header file.

	@author Ian Wilkey (iwilkey)
	@date-created 8/30/2022
*/

#ifndef _HAL_AVR_H_
#define _HAL_AVR_H_

#include <stdint.h>

// Quick component codes...
#define BUTTON_0 (0x01)
#define BUTTON_1 (0x02)
#define BUTTON_2 (0x04)
#define BUTTON_3 (0x08)
#define LED_0 (0x01)
#define LED_1 (0x02)
#define LED_2 (0x04)
#define LED_3 (0x08)
#define LED_4 (0x10)
#define LED_5 (0x20)

enum IOType {
	LEDS,
	BUTTONS,
	PIB, // PINB
	PIC, // PINC
	PID, // PIND
	PORB, // PORTB
	PORC, // PORTC
	PORD // PORTD
};

// IO utilities...
uint8_t getStateOf(enum IOType type);
void setStateOf(enum IOType type, uint8_t value);

// Behavioral utilities...
void delayMS(unsigned short int ms);

// Inline logical utilities...
inline uint8_t getChangeBetween(uint8_t first, uint8_t second, uint8_t mask) { return ((first ^ second) & second & mask); }
inline uint8_t getToggledBitWith(uint8_t first, uint8_t second) { return first ^ second; }

void init(void);
void dispose(void);

#endif // _HAL_AVR_H_
