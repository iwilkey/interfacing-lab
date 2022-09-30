////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file ECE-322/Lab4/solution.c
/// @brief This embedded C program operates to satisfy the requirements of ECE 322 Lab 4.
/// @authors Ian Wilkey, Andrew Kamp, Rachel Gottschalk
/// @date 9/20/2022
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _ECE_322_LAB_4_SOLUTION_C_
#define _ECE_322_LAB_4_SOLUTION_C_

// Include required libraries...
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Hardware abstraction libraries created by Dr. Mali...
#include "bios_kpads.h"
#include "bios_leds.h"
#include "bios_timer_int.h"

#define MOTOR_TIMER_FREQ 5000
#define MOTOR_PWM_FREQ 500
// This function is the Timer1 PWM generator for the motor AND it's behavior...
void motorControl(void) {

	// Interrupt cycle counter...
	static uint8_t c = 0;
	// Check to see if the program should even generate the PWM ('C') key...
	static bool on = true;
	// This will be used to render the LED for the ocilloscope timing of interrupt. 
	static uint8_t checkState = 1;
	// This will hold the current PWM state.
	static uint8_t pinState = 0;
	// This is the duty cycle variable.
	static double dutyCycle = 0.0f;

	// If the PWM should be generated...
	if(on) {
		// Do simple calculation to see where we are in the PWM and set the LOGIC LEVEL right.
		if(c >= (MOTOR_TIMER_FREQ / MOTOR_PWM_FREQ) 
			* (dutyCycle / 100.0f)) pinState = 0;
		else pinState = 1;
		c++;
		c %= (MOTOR_TIMER_FREQ / MOTOR_PWM_FREQ);
	}

	// See if a key is being pressed...
	uint8_t key = keypressed();
	// If that key is between 0 and A, it is a speed control...
	if(key >= 0 && key <= 0x0A) {
		on = true;
		// Set the speed control based on equation calculated below...
		dutyCycle = ((100.0f / 11.0f) * key) + 15.0f;
	  // If the key is C, turn off PWM gen.
	} else if(key == 0x0C) on = false;

	// Render to pin...
	leds_set(((pinState) ? B_L0 : 0x00));

	// Toggle check state for scope reading...
	leds_set(leds_get() | ((checkState) ? B_L4 : 0));
	checkState = (checkState) ? 0 : 1;
}

// NOTE, this function behaves ABOUT the same as the one above, so comments are ommitted.
#define SERVO_TIMER_FREQ 10000
#define SERVO_PWM_FREQ 50
void servoControl(void) {
	static uint8_t c = 0;
	static bool on = true;
	static uint8_t checkState = 1;
	static uint8_t pinState = 0;
	static double dutyCycle = 0.5f;
	if(on) {
		if(c > (SERVO_TIMER_FREQ / SERVO_PWM_FREQ) 
			* (dutyCycle / 100.0f)) pinState = 0;
		else pinState = 1;
		c++;
		c %= (SERVO_TIMER_FREQ / SERVO_PWM_FREQ);
	}
	uint8_t key = keypressed();
	if(key >= 0 && key <= 0x0A) {
		on = true;
		dutyCycle = (((12.5f - 0.5f) / 11.0f) * key) + 1.5f;
	} else if(key == 0x0C) on = false;
	leds_set(((pinState) ? B_L0 : 0x00));
	leds_set(leds_get() | ((checkState) ? B_L4 : 0));
	checkState = (checkState) ? 0 : 1;
}

int main(void) {

	// IO config...
	
	// Setting PORTD as a OUTPUT
	DDRD = 0xff; // OUTPUT...
	PORTD = 0x00; // pull ups...
	PIND = 0x00; // Set output as ON...
	// Set PORTC as a INPUT...
	DDRC = 0x00; // INPUT...
	PORTC = 0xff; // pull ups...
	PINC = 0xff;
	leds_init();

	// Pick which device we are testing...
	// Timer1_initialize(SERVO_TIMER_FREQ, servoControl, timer_prescale_1); // Servo
	Timer1_initialize(MOTOR_TIMER_FREQ, motorControl, timer_prescale_1); // Motor
	sei();
    while(1) {}
}

#endif // _ECE_322_LAB_4_SOLUTION_C_
