#ifndef _I2C_LIB_CPP_
#define _I2C_LIB_CPP_

#include <avr/io.h>

class I2C {
    public:

        I2C() {
            // Set SCL to 400 kHz, and we don't need need a prescaler...
            TWSR = 0x00;
            TWBR = 0x0C;
            // Enable TWI...
            TWCR = (1 << TWEN);
        }

        ~I2C() {}

        void start(uint8_t addr) {
            TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
            // Wait for status (until TWINT resets zero)...
            while(!(TWCR & (1 << TWINT)));
            write(addr);
        }

        inline void stop(void) {
            TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
        }

        void write(uint8_t data) {
            TWDR = data;
            TWCR = (1 << TWINT) | (1 << TWEN);
            while(!(TWCR & (1 << TWINT)));
        }

        uint8_t read(bool ack) {
            TWCR = (1 << TWINT) | (1 << TWEN);
            if(ack) 
                TWCR |= (1 << TWEA);
            while(!(TWCR & (1<<TWINT)));
            return TWDR;
        }

        uint8_t getStatus(void) {
            uint8_t status;
            status = TWSR & 0xF8;
            return status;
        }

};

#endif // _I2C_LIB_CPP_
