#ifndef _LiquidCrystal1602_LIB_CPP_
#define _LiquidCrystal1602_LIB_CPP_

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define DATA_BUS PORTC
#define COMMAND_BUS PORTB
#define LCD_RS 0 // PORT B0
#define LCD_RW 1 // PORT B1
#define LCD_E 2 // PORT B2

typedef enum { 
    LCD_COMMAND_REGISTER = 0, 
    LCD_DATA_REGISTER = 1
} lcdRegister_t;

class LiquidCrystal1602 {
    public:

        LiquidCrystal1602() {
            // Set up ports...
            DDRB = 0b00000111;
            PORTB = ~DDRB;
            DDRC = 0x0f;
            PORTC = ~DDRC;
            _delay_ms(20);
            // Set LCD to 4-bit mode...
            putCommand(0b00111100);
            // Enable 5x7 mode for chars...
            putCommand(0x28);
            // Set cursor on and to blink...
            cursor(true);
            // Clear display...
            clear();
            // Move the cursor to beginning of first line...
            home();
        }

        ~LiquidCrystal1602() {}

        inline void putChar(char chr) {
            putNibble(LCD_DATA_REGISTER, chr);
        }

        inline void putCommand(uint8_t command) {
            putNibble(LCD_COMMAND_REGISTER, command);
        }

        inline void clear(void) {
            putCommand(0x01);
        }

        inline void home(void) {
            putCommand(0x80);
        }

        inline void cursor(bool verdict) {
            putCommand(((verdict) ? 0b00001111 : 0b00001100));
        }

        void print(const char * str) {
            while(*str != '\0') {
                putChar(*str);
                ++str;
            }
        }

        void print(uint32_t number) {
            char str[32];
            sprintf(str, "%u", number);
            for(int i = 0; i < 32; i++) {
                if((str[i] < '0' || str[i] > '9'))
                    if(str[i] != '-')
                        continue;
                putChar(str[i]);
            }
        }

        void setCursor(uint8_t x, uint8_t y) {
            home();
            if(y != 1) 
                putCommand(0xc0);
            while(x != 1) {
                putCommand(0b00010100);
                x--;
            }
        }

    private:

        void wait(void) {
            uint8_t read;
            while(true) {
                read = getDataNibble();
                // Read D0 (MSB of read dat nibble = BF) to see if it is low (not busy), otherwise
                // loop till it is!
                if(!(read & 0x80)) break;
            }
            return;
        }

        void putNibble(lcdRegister_t reg, uint8_t data) {
            wait();
            switch(reg) {
                case LCD_COMMAND_REGISTER:
                    // Send upper nibble of data, pull enable high...
                    COMMAND_BUS |= 1 << LCD_E;
                    DATA_BUS = (data >> 4) & 0x0f;
                    // Pull enable low to get ready for lower nibble...
                    COMMAND_BUS &= ~(1 << LCD_E);
                    // Send lower nubble of data, pull enable high again...
                    COMMAND_BUS |= 1 << LCD_E;
                    DATA_BUS = data & 0x0f;
                    // Reset enable...
                    COMMAND_BUS &= ~(1 << LCD_E);
                    break;
                case LCD_DATA_REGISTER:
                    // Pull enable high and register select DATA (RS -> 1)...
                    COMMAND_BUS |= (1 << LCD_E | 1 << LCD_RS);
                    // Gather upper nibble...
                    DATA_BUS = (data >> 4) & 0x0f;
                    COMMAND_BUS &= ~(1 << LCD_E);
                    COMMAND_BUS |= (1 << LCD_E);
                    // Gather lower nibble...
                    DATA_BUS = data & 0x0f;
                    // PULL RS to 0! If this is not done, reading data will NOT work correctly.
                    COMMAND_BUS &= ~(1 << LCD_E | 1 << LCD_RS);
                    break;
                default:;
            }
        }

        uint8_t getDataNibble(void) {
            // Init a variable to hold read data...
            uint8_t out = 0x00;

            // Set C to INPUT to read LCD data...
            DDRC = 0b11110000;
            // No pull up resistors...
            PORTC = DDRC;

            // Pull READ/WRITE high (read)...
            COMMAND_BUS |= 1 << LCD_RW;
            // Wait tSP1 >= 30ns (1us is OK)...
            _delay_us(1);   

            // Pull enable high...
            COMMAND_BUS |= 1 << LCD_E;
            // Wait two cycles...
            asm("nop");
            asm("nop");
            // Collect upper nibble of data...
            out = PINC;
            // Pull enable low to get ready to read lower nibble...
            COMMAND_BUS &= ~(1 << LCD_E);

            // Shift the current data read to higher nibble portion and clear lower nibble...
            out <<= 4;
            out &= 0xf0;

            // Collect lower nibble...
            COMMAND_BUS |= 1 << LCD_E;
            asm("nop");
            asm("nop");
            // Place the lower nibble in variable without disturbing upper nibble...
            out |= (PINC & 0x0f);
            COMMAND_BUS &= ~(1 << LCD_E);
            // Wait tHD2 >= 10ns (1us is OK)...
            _delay_us(1);
            // Wait tHD1 >= 10ns (1us is OK)...
            COMMAND_BUS &= ~(1 << LCD_RW);
            _delay_us(1);

            // Set C back to output...
            DDRC = 0x0f;
            PORTC = ~DDRC;
            PINC = 0x00;

            return out;
        }

};

#endif // _LiquidCrystal1602_LIB_CPP_
