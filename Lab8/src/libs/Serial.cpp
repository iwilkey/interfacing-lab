#ifndef _SERIAL_LIB_CPP_
#define _SERIAL_LIB_CPP_

#include <avr/io.h>

#define xtal 16000000L

class Serial {
    public:

        Serial(uint16_t baud) {
            this->baud = baud;
            uint32_t temp = (xtal / 16) / baud - 1;
            UBRR0H = (temp >> 8) & 0x0F;
            UBRR0L = (temp & 0xFF);
            UCSR0B = (1 << RXEN0) | (1 << TXEN0);
            UCSR0C = (3 << UCSZ00) | (1 << USBS0);
        }

        ~Serial() {
            UCSR0B = 0;
        }

        inline uint16_t getBaud() {
            return this->baud;
        }

        inline void print(const char * const s) {
            putString(s);
        }

        void println(const char * const s) {
            putString(s);
            putChar('\n');
        }

        void println(int number) {
            char str[32];
            sprintf(str, "%d", number);
            putString(str);
            putChar('\n');
        }

    private:

        int baud;
        
        inline uint8_t readyTX(void) {
            return (0 != (UCSR0A & 1 << UDRE0));
        }

        inline uint8_t readyRX(void) {
            return (0 != (UCSR0A & 1 << RXC0));
        }

        inline uint8_t checkError(void) {
            return (UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0)));
        }

        char getChar(void) {
            while(0 == (UCSR0A & 1 << RXC0));
            return (uint8_t)UDR0;
        }

        void putString(const char * const s) {
            for(const char * p = s; *p != '\0'; ++p)
                putChar(*p);
        }

        void putChar(char c) {
            while(0 == (UCSR0A & 1 << UDRE0));
            UDR0 = c;
        }

};

#endif // _SERIAL_LIB_CPP_