#include <avr/io.h>
#include <avr/interrupt.h>
#include "bios_kpads.h"
#include "bios_leds.h"

// This function will wait for a key to be pressed, turn on the white LED while key pressed runs, then exit right when it finishes.
// The idea is to hook the LED to the scope and measure the time the voltage is high.
void testTimeInKeyPressed(void) {
	leds_set(0b00000000);
	if(keypressed() != KEY_NONE) {
		leds_set(0b00010000);
		leds_set(leds_get() & (keypressed() & 0b11101111));
		leds_set(0x00);
		exit(0);
	}
}

void keyPressedTask(void) {
	while(1) {
		// testTimeInKeyPressed();
		leds_set(keypressed());
	}
}

void getKeyTask(void) {
	while(1) {
		leds_set(getkey());
	}
}

void init(void) {
	
	// Setting PORTD as a OUTPUT
	DDRD = 0xFF; // OUTPUT...
	PORTD = 0b11111111; // pull ups...
	PIND = 0xFF; // Set output as ON...
	
	// Set PORTC as a INPUT...
	DDRC = 0x00; // Input...
	PORTC = 0b11111111; // pull ups...
	
	// Set up PORTC for pin change interrupt...
	PCIFR = PCIFR & ~(1 << PCIF1); // clear pending interrupts if any
	PCICR = PCICR | 1 << PCIE1; // enable group PINC PINC interrupts
	PCMSK1 = 0b11111111;
	
	leds_init();
}

int main(void) {
	init();
	sei();
	while(1) {}
    return(0);
}

ISR(PCINT1_vect) {
	leds_set(0b00010000 | getkey());
	leds_set(leds_get() & 0b11101111);
}
