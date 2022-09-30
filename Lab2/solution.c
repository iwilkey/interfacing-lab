////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file ECE-322/Lab2/solution.c
/// @brief This embedded C program operates to satisfy the requirements of ECE 322 Lab 2.
/// @authors Ian Wilkey, Andrew Kamp, Rachel Gottschalk
/// @date 9/2022
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _ECE_322_LAB_2_SOLUTION_C_
#define _ECE_322_LAB_2_SOLUTION_C_

// Include required libraries...
#include <avr/io.h>
#include <avr/interrupt.h>

// Hardware abstraction libraries created by Dr. Mali.
#include "bios_keys.h"
#include "bios_leds.h"
#include "library.h"

// ISR function handle.
static void (*PCINT_PIND_function)(void) = 0;

// Render LED with state of corresponding button in PIND pin change interrupt.
void interruptA(void) {
	leds_set(keys_get());
	return;
}

// Toggle LED state with corresponding button in PIND pin change interrupt.
void interruptB(void) {
	leds_set(leds_get() | B_L4);

	static uint8_t buttonsCurr;
	static uint8_t buttonsOld;
	static uint8_t ledCurr;
	static uint8_t first;
	static uint8_t eventCounter;

	if(!first) {
		buttonsOld = keys_get();
		first = 1;
	}

	buttonsCurr = keys_get();
	if(eventCounter % 2 == 0) {
		ledCurr = toggle_bit(ledCurr, check_button(
			buttonsOld, buttonsCurr, 0b11111111
			)
		);
	}
	buttonsOld = buttonsCurr;

	leds_set(ledCurr);
	eventCounter++;
	eventCounter %= 2;

	delay(10); // Prevent bouncing... (otherwise, need a capacitor).
	return;
}

// Entry point here.
int main(void) {
	leds_init();
	keys_init();
	// PCINT_PIND_function = interruptA;
	PCINT_PIND_function = interruptB;
	// Clear interrupts and set PIND to trigger pin change interrupt.
	PCIFR = PCIFR & ~(1 << PCIF1);
	PCICR = PCICR | 1 << PCIE2;
	PCMSK2 = 0b11111111;
	sei();
	while(1) {} // No need for anything in the while loop, everything happens in the ISR.
	return 0;
}

// Interrupt service routine for PCINT on PIND; trigger function described in void handle.
ISR(PCINT2_vect) {
	PCINT_PIND_function();
}

#endif // _ECE_322_LAB_2_SOLUTION_C_
