#include <avr/io.h>
#include "library.h"
#include "bios_leds.h"

// Checkpoint 1-2
void checkpointA(void) {
	uint8_t time = 0;
	while(1) {
		time++;
		if(time >= 0 && time <= 1) leds_set(0b11111111);
		else if(time > 1 && time < 3) leds_set(0b00000000);
		else time = 0;
		delay(1000);
	}
}

// Checkpoint 1-3
void checkpointB(void) {
	uint8_t ledPattern[5] = { B_L0, B_L1, B_L2, B_L3, B_L4 },
	ledIndex = 0;
	while(1) {
		leds_set(0b00000000);
		leds_set(ledPattern[ledIndex]);
		ledIndex++;
		ledIndex %= 5;
		delay(1000);
	}
}

// Checkpoint 1-4
void checkpointC(void) {
	while(1) {
		uint8_t keys = keys_get();
		leds_set(keys);
	}
}

// Checkpoint 1-5
void checkpointD(void) {
	uint8_t buttonsCurr, buttonsOld,
		ledCurr = 0;
	buttonsOld = keys_get();
	while(1) {
		buttonsCurr = keys_get();
		ledCurr = toggle_bit(ledCurr, 
			check_button(buttonsOld, buttonsCurr, 0b00001111));
		buttonsOld = buttonsCurr;
		leds_set(ledCurr);
		delay(100);
	}
}

// Testing main method (comment out ones not testing).
int main(void) {
	leds_init();
	keys_init();
	// checkpointA();
	// checkpointB();
	// checkpointC();
	// checkpointD();
}
