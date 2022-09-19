#include <avr/io.h>
#include <avr/interrupt.h>
#include "bios_kpads.h"
#include "bios_leds.h"
#include "bios_timer_int.h"

void init(void) {
	// Setting PORTD as a OUTPUT
	DDRD = 0xFF; // OUTPUT...
	PORTD = 0b11111111; // pull ups...
	PIND = 0xFF; // Set output as ON...
	// Set PORTC as a INPUT...
	DDRC = 0x00; // INPUT...
	PORTC = 0b11111111; // pull ups...
	leds_init();
}

#define SERVO_TIMER_FREQ 10000
#define SERVO_PWM_FREQ 50
void servoControl(void) {
	static uint8_t c = 0, 
		on = 1,
		checkState = 1,
		pinState = 0;
	static double dutyCycle = 0.5f;
	if(on) {
		if(c > (SERVO_TIMER_FREQ / SERVO_PWM_FREQ) * (dutyCycle / 100.0f)) pinState = 0;
		else pinState = 1;
		c++;
		c %= (SERVO_TIMER_FREQ / SERVO_PWM_FREQ);
	}
	uint8_t key = keypressed();
	if(key >= 0 && key <= 0x0A) {
		on = 1;
		dutyCycle = (((12.5f - 0.5f) / 11.0f) * key) + 1.5f;
	} else if(key == 0x0C) on = 0;
	leds_set(((pinState) ? B_L0 : 0x00));
	leds_set(leds_get() | ((checkState) ? B_L4 : 0));
	checkState = (checkState) ? 0 : 1;
}

#define MOTOR_TIMER_FREQ 5000
#define MOTOR_PWM_FREQ 500
void motorControl(void) {
	static uint8_t c = 0,
		on = 1,
		checkState = 1,
		pinState = 0;
	static double dutyCycle = 0.0f;
	if(on) {
		if(c >= (MOTOR_TIMER_FREQ / MOTOR_PWM_FREQ) * (dutyCycle / 100.0f)) pinState = 0;
		else pinState = 1;
		c++;
		c %= (MOTOR_TIMER_FREQ / MOTOR_PWM_FREQ);
	}
	uint8_t key = keypressed();
	if(key >= 0 && key <= 0x0A) {
		on = 1;
		dutyCycle = ((100.0f / 11.0f) * key) + 15.0f;
	} else if(key == 0x0C) on = 0;
	leds_set(((pinState) ? B_L0 : 0x00));
	leds_set(leds_get() | ((checkState) ? B_L4 : 0));
	checkState = (checkState) ? 0 : 1;
}

int main(void) {
	init();
	// Timer1_initialize(SERVO_TIMER_FREQ, servoControl, timer_prescale_1);
	Timer1_initialize(MOTOR_TIMER_FREQ, motorControl, timer_prescale_1);
	sei();
    while(1) {}
}
