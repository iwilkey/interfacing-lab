////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file ECE-322/Lab5/solution.c
/// @brief This embedded C program operates to satisfy the requirements of ECE 322 Lab 5.
/// @authors Ian Wilkey, Andrew Kamp, Rachel Gottschalk
/// @date 9/29/2022
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _ECE_322_LAB_5_SOLUTION_C_
#define _ECE_322_LAB_5_SOLUTION_C_

// Include required libraries...
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

// Define the frequency of the CPU (as needed for the Timer1 Interrupt configuration).
#define FREQ_CPU 16000000L

// This function will set the CPU SLEEP MODE to an IDLE STATE (low power consumption).
void sleepOn(void) {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_cpu();
}

// Timer1 semaphore...
volatile bool flag = false;
// On Timer1...
ISR(TIMER1_COMPA_vect) {
    // At 16 hz (timer frequncy), we must count to 8 to do the inside function
    // at 2 hz...
    static uint8_t c = 0;
    if(c >= 8) {
        // This will toggle on and off the LED2 (debug) LED...
        static bool check = false;
        if(check) PORTB |= (1 << 2);
        else PORTB &= ~(1 << 2);
        check = !check;

        // Enable the semaphore....
        flag = true;
        // Disable sleep mode (The CPU has to do something...)
        sleep_disable();
        c = 0;
    }
    c++;
}

int main(void) {

    // B0 and B1 5V OUTPUT...
    DDRB = 0b00000111;
    // Enable Pull-up resistors on INPUT pins, pull OUTPUT pins low...
    PORTB = ~DDRB;
    // No need for PINB, all USED pins are OUTPUTS...
    PINB = 0x00;

    // D5 (Checking HI or LOW) INPUT, D4 (RESET) INPUT...
    DDRD = 0b11001111;
    // Turn off ALL PULLUPS because we already have resistors in the circuit. If
    // they were on, that would be overkill and may even cause problems getting
    // a valid LOGIC HIGH...
    PORTD = 0x00;
    // Any USED pins on D are INPUTS...
    PIND = 0xff;

    // Configure Timer1...
                                     // -  1  8 64 256 1024
    const static uint8_t prescales[] = { 0, 0, 3, 6, 8, 10 };
    TCNT1 = 0x00;
    OCR1A = ((uint32_t)(FREQ_CPU) >> prescales[4]) / 1L - 1;
    TCCR1A = 0x00;
    TCCR1B = 0x08 | (0x07 & prescales[5]);
    TIFR1  = 0;
    TIMSK1 = (1 << OCIE1A);
    sei();
    
    // Configure the Watchdog timer (7 ~= 1.8 (2) seconds)...
    wdt_enable(7);

    // Loop counter...
    uint8_t lc = 10;
    while(true) {

        // While the timer overflow hasn't been set yet, reset the watchdog and put
        // the device into sleep mode.
        while(!flag) {
            wdt_reset();
            sleepOn();
        }
        flag = false;
        // When loop count equals zero...
        if(!lc) {
            // Preform the "fire check"...

            // Turn the LEDsens ON
            PORTB |= (1 << 1);
            // Wait about 5 ms for the photoresistor voltage to settle...
            _delay_ms(5);
            // Check to see if the input on D5 is a LOGIC HIGH...
            if((((~PIND) >> 4) & 0x0f) == (1 << 1)) {
                // Fire detected!
                while(true) {
                    // Reset the watchdog...
                    wdt_reset();

                    // Sound the alarm!
                    PORTB |= (1 << 0);
                    // The time here controls the frequency of the alarm...
                    _delay_ms(2);
                    // Turn off the alarm so as to resound next loop...
                    PORTB &= ~(1 << 0);
                    _delay_ms(4);

                    // The only way to get out of this fire alarm loop is to hit
                    // D4, the RESET button...
                    if((((~PIND) >> 4) & 0x0f) == (1 << 0)) break; 
                }
            }
            // Turn OFF LEDsens.
            PORTB &= ~(1 << 1);
            _delay_ms(20);
            // Reset loop counter...
            lc = 10;
        } else lc--;
    }

    return 0;
}

#endif // _ECE_322_LAB_5_SOLUTION_C_
