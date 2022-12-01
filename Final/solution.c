////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file ECE-322/Final/solution.c
/// @brief This firmware operates to satisfy the requirements of ECE 322 Lab 8 (final project).
/// @authors Ian Wilkey, Andrew Kamp, Rachel Gottschalk
/// @date 11/27/2022
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _ECE_322_FINAL_SOLUTION_C_
#define _ECE_322_FINAL_SOLUTION_C_

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>

#define xtal 16000000L

    ////////////////////////////////////
    ////////////////////////////////////
    ////////////////////////////////////
    ///       FIRMWARE SECTION       ///
    ////////////////////////////////////
    ////////////////////////////////////
    ////////////////////////////////////

// UART library header...
#ifndef _UART_LIB_H_
#define _UART_LIB_H_

    #define UART_BPS_2400 2400
    #define UART_BPS_4800 4800
    #define UART_BPS_9600 9600
    #define UART_BPS_19200 19200
    #define UART_BPS_38400 38400
    #define UART_BPS_57600 57600
    #define UART_BPS_115200 115200
    #define UART_STOP_ONE 0
    #define UART_STOP_TWO 1
    #define UART_PARITY_NONE 0
    #define UART_PARITY_EVEN 2
    #define UART_PARITY_ODD 3

        void uart0_initialize(uint16_t baud);
        void uart0_initialize3(uint16_t baud, uint8_t uart_stop_mode, uint8_t uart_parity_mode);
        void uart0_shutdown();
        uint8_t uart0_ready_TX();
        void uart0_putc(char c);
        void uart0_puts(const char * const s);
        uint8_t uart0_ready_RX();
        char uart0_getc();
        char uart0_getc_echo();
        uint8_t uart0_check_error();
        size_t uart0_gets(char * s, size_t size);
        size_t uart0_gets_echo(char * s, size_t size);
        size_t uart0_gets_edit(char * s, size_t size);
        size_t uart0_read(void * s, size_t size);
        size_t uart0_write(const void * const s, size_t size);

#endif // _UART_LIB_H_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// UART Implementation...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void uart0_initialize(uint16_t baud) {
    uint32_t temp = (xtal / 16) / baud - 1;
    UBRR0H = (temp >> 8) & 0x0F;
    UBRR0L = (temp & 0xFF);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (3 << UCSZ00) | (1 << USBS0);
}

void uart0_initialize3(uint16_t baud, uint8_t uart_stop_mode, uint8_t uart_parity_mode) {
    uint32_t temp = xtal / 16 / baud - 1;
    UBRR0H = (temp >> 8) & 0x0F;
    UBRR0L = (temp & 0xFF);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (3 << UCSZ00) | (1 << USBS0);
}

void uart0_shutdown(void) {
    UCSR0B = 0;
}

uint8_t uart0_ready_TX(void) {
    return 0 != (UCSR0A & 1<<UDRE0);
}

void uart0_putc(char c) {
    while(0 == (UCSR0A & 1 << UDRE0));
    UDR0 = c;
}

void uart0_puts(const char * const s) {
    for(const char * p = s; *p != '\0'; ++p)
        uart0_putc(*p);
}

uint8_t uart0_ready_RX(void) {
    return 0 != (UCSR0A & 1 << RXC0);
}

char uart0_getc(void) {
    while(0 == (UCSR0A & 1 << RXC0));
    return (uint8_t)UDR0;
}

char uart0_getc_echo(void) {
   char c = uart0_getc();
   uart0_putc(c);
   return c;
}

uint8_t uart0_check_error(void) {
    return UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0));
}

size_t uart0_gets_echo(char * s, size_t size) {
    char c;
    size_t count = 0;
    --size;
    while(true) {
        c = uart0_getc();
        if(c == '\b' || c == 127) {
            uart0_putc('\a');
        } else if(c == '\n' || c == '\r') {
            uart0_putc('\r');
            uart0_putc('\n');
            *s = 0;
            break;
        } else {
            if(count < size) {
                *s++ = c;
                ++count;
                uart0_putc(c);
            } else {
                uart0_putc('\a');
            }
        }
    }
    return count;
}

size_t uart0_gets (char * s, size_t size) {
    char c;
    size_t count = 0;
    --size;
    while(true) {
        c = uart0_getc_echo();
        if(c == '\n' || c == '\r' || count >= size) 
            break;
        *s++ = c;
        ++count;
    }
    *s = 0;
    return count;
}

size_t uart0_gets_edit(char * s, size_t size) {
    char c;
    size_t count = 0;
    --size;
    while(true) {
        c = uart0_getc();
        switch(c) {
            case '\b':
            case 127:
                if(count) {
                    uart0_putc('\b');
                    uart0_putc(' ');
                    uart0_putc('\b');
                    --s;
                    --count;
                }
                break;
            case '\n':
            case '\r':
                uart0_putc('\r');
                uart0_putc('\n');
                *s = 0;
                return count;
                default:
                if(count < size) {
                    *s++ = c;
                    ++count;
                    uart0_putc(c);
                } else uart0_putc('\a');
                break;
        }
    }
    return count;
}

size_t uart0_write(const void * const s, size_t size) {
    const uint8_t * p = s;
    size_t i = 0;
    while(i < size) {
        uart0_putc(*p);
        ++p;
        ++i;
    }
    return i;
}

size_t uart0_read(void * s, size_t size) {
    uint8_t * p = s;
    char c;
    size_t count = 0;
    while(count < size) {
        c = uart0_getc();
        *p++ = c;
        ++count;
    }
    return count;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Utilities header...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _UTIL_LIB_H_
#define _UTIL_LIB_H_

    char * u16bts(uint16_t num, char buffer[]);

#endif // _UTIL_LIB_H_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// TWI header...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _TWI_LIB_H_
#define _TWI_LIB_H_

    void twiInit(void);
    void twiStart(uint8_t addr);
    void twiStop(void);
    void twiWrite(uint8_t data);
    uint8_t twiAckRead(void);
    uint8_t twiNoAckRead(void);
    uint8_t twiGetStatus(void);

#endif // _TWI_LIB_H_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// LCD header...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _LCD_LIB_H_
#define _LCD_LIB_H_

    #define DATA_BUS PORTC
    #define COMMAND_BUS PORTB
    #define LCD_RS 0 // PORT B0
    #define LCD_RW 1 // PORT B1
    #define LCD_E 2 // PORT B2

        typedef enum { 
            LCD_COMMAND_REGISTER = 0, 
            LCD_DATA_REGISTER = 1
        } lcdRegister_t;

        void lcdPutNibble(lcdRegister_t reg, uint8_t data);
        uint8_t lcdGetDataNibble(void);
        void lcdInit(void);
        void lcdPutChar(char chr);
        void lcdPutString(const char * string);
        void lcdPutCommand(uint8_t command);
        void lcdClear(void);
        void lcdHome(void);
        void lcdCursor(bool verdict);
        void lcdSetCursorXY(uint8_t x, uint8_t y);
        void lcdWait(void);

#endif // _LCD_LIB_H_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MAX30102 header...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _MAX30102_LIB_H_
#define _MAX30102_LIB_H_

    #define MAX_WRITE_ADDR 0xAE
    #define MAX_READ_ADDR 0xAF
    #define REG_INTR_STATUS_1 0x00
    #define REG_INTR_STATUS_2 0x01
    #define REG_INTR_ENABLE_1 0x02
    #define REG_INTR_ENABLE_2 0x03
    #define REG_FIFO_WR_PTR 0x04
    #define REG_OVF_COUNTER 0x05
    #define REG_FIFO_RD_PTR 0x06
    #define REG_FIFO_DATA 0x07
    #define REG_FIFO_CONFIG 0x08
    #define REG_MODE_CONFIG 0x09
    #define REG_SPO2_CONFIG 0x0A
    #define REG_LED1_PA 0x0C
    #define REG_LED2_PA 0x0D
    #define REG_PILOT_PA 0x10
    #define REG_MULTI_LED_CTRL1 0x11
    #define REG_MULTI_LED_CTRL2 0x12
    #define REG_TEMP_INTR 0x1F
    #define REG_TEMP_FRAC 0x20
    #define REG_TEMP_CONFIG 0x21
    #define REG_PROX_INT_THRESH 0x30
    #define REG_REV_ID 0xFE
    #define REG_PART_ID 0xFF
    #define SAMPLE_PACKET_SIZE 50

        // Data type that holds sample data from the red LED and the ir LED...
        struct maxSample_t {
            uint16_t red;
            uint16_t ir;
        };

        // TWI methods...
        void maxInit(void);
        void maxReadFifo(uint16_t * red, uint16_t * ir);
        void maxWriteRegister(uint8_t toAddr, uint8_t data);
        void maxReadRegister(uint8_t fromAddr, uint8_t * intoData);
        void maxReset(void);

        // Application methods...
        void maxConvertData(uint16_t * irBuffer, uint16_t * redBuffer, int32_t * dataOut);
        
#endif // _MAX30102_LIB_H_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MAX30102 implementation...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _MAX30102_LIB_C_
#define _MAX30102_LIB_C_

void maxInit(void) {
    // INTR setting...
    maxWriteRegister(REG_INTR_ENABLE_1, 0xc0);
    maxWriteRegister(REG_INTR_ENABLE_2, 0x00);
    // FIFO_WR_PTR[4:0]...
    maxWriteRegister(REG_FIFO_WR_PTR, 0x00);
    // OVF_COUNTER[4:0]...
    maxWriteRegister(REG_OVF_COUNTER, 0x00);
    // FIFO_RD_PTR[4:0]...
    maxWriteRegister(REG_FIFO_RD_PTR, 0x00);
    // Sample avg: 4, FIFO rollover=false, FIFO almost full: 17...
    maxWriteRegister(REG_FIFO_CONFIG, 0x4f);
    // 0x02 for red only, 0x03 for SpO2 mode 0x07 multimode LED...
    maxWriteRegister(REG_MODE_CONFIG, 0x03);
    // SPO2_ADC range: 4096 nA, SPO2 sample rate: 100 Hz, LED pulse width: 411uS...
    maxWriteRegister(REG_SPO2_CONFIG, 0x27);
    // Choose value for ~ 7mA for LED1...
    maxWriteRegister(REG_LED1_PA, 0x24);
    // Choose value for ~ 7mA for LED2...
    maxWriteRegister(REG_LED2_PA, 0x24);
    // Choose value for ~ 25mA for pilot LED...
    maxWriteRegister(REG_PILOT_PA, 0x7f);
}

// Read a set of samples from the MAX30102 FIFO register...
void maxReadFifo(uint16_t * red, uint16_t * ir) {

    *red = 0x00;
    *ir = 0x00;

    uint8_t intrState;
    maxReadRegister(REG_INTR_STATUS_1, &intrState);
    maxReadRegister(REG_INTR_STATUS_2, &intrState);

    twiStart(MAX_WRITE_ADDR);
    twiWrite(REG_FIFO_DATA);
    twiStop();

    twiStart(MAX_READ_ADDR);

    uint32_t dataBuffer;

    // Read into red...
    dataBuffer = twiAckRead();
    dataBuffer <<= 16;
    *red += dataBuffer;
    dataBuffer = twiAckRead();
    dataBuffer <<= 8;
    *red += dataBuffer;
    dataBuffer = twiAckRead();
    *red += dataBuffer;

    // Read into ir...
    dataBuffer = twiAckRead();
    dataBuffer <<= 16;
    *ir += dataBuffer;
    dataBuffer = twiAckRead();
    dataBuffer <<= 8;
    *ir += dataBuffer;
    dataBuffer = twiNoAckRead();
    *ir += dataBuffer;

    twiStop();

    *red &= 0x03ffff;
    *ir &= 0x03ffff;

}

void maxWriteRegister(uint8_t toAddr, uint8_t data) {
    twiStart(MAX_WRITE_ADDR);
    twiWrite(toAddr);
    twiWrite(data);
    twiStop();
}

void maxReadRegister(uint8_t fromAddr, uint8_t * intoData) {
    twiStart(MAX_WRITE_ADDR);
    twiWrite(fromAddr);
    twiStop();
    twiStart(MAX_READ_ADDR);
    *intoData = twiNoAckRead();
    twiStop();
}

void maxReset(void) {
    maxWriteRegister(REG_MODE_CONFIG, 0x40);
}

void maxConvertData(uint16_t * irBuffer, uint16_t * redBuffer, int32_t * dataOut) {
    // Calculate DC mean...
    uint32_t irMean = 0;
    for(int i = 0; i < SAMPLE_PACKET_SIZE; i++) 
        irMean += irBuffer[i];
    irMean /= SAMPLE_PACKET_SIZE;
    // Invert...
    for(int i = 0; i < SAMPLE_PACKET_SIZE; i++)
        dataOut[i] = -1 * (irBuffer[i] - irMean);
    int npoint = 16;
    // Perform n-pt moving average...
    for(int i = 0; i < SAMPLE_PACKET_SIZE; i++) {
        int indArr[npoint];
        for(int j = 0; j < npoint; j++)
            indArr[npoint] = (i + j) % SAMPLE_PACKET_SIZE;
        int ls = 0;
        for(int j = 0; j < npoint; j++)
            ls += irBuffer[indArr[npoint]];
        ls /= npoint;
        dataOut[i] = ls;
    }
}

#endif // _MAX30102_LIB_C_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// LCD implementation...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _LCD_LIB_C_
#define _LCD_LIB_C_

void lcdPutNibble(lcdRegister_t reg, uint8_t data) {
    lcdWait();
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

uint8_t lcdGetDataNibble(void) {
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

void lcdWait(void) {
    uint8_t read;
    while(true) {
        read = lcdGetDataNibble();
        // Read D0 (MSB of read dat nibble = BF) to see if it is low (not busy), otherwise
        // loop till it is!
        if(!(read & 0x80)) break;
    }
    return;
}

inline void lcdPutChar(char chr) {
    lcdPutNibble(LCD_DATA_REGISTER, chr);
}

inline void lcdPutCommand(uint8_t command) {
    lcdPutNibble(LCD_COMMAND_REGISTER, command);
}

inline void lcdClear(void) {
    lcdPutCommand(0x01);
}

inline void lcdHome(void) {
    lcdPutCommand(0x80);
}

void lcdPutString(const char * str) {
    while(*str != '\0') {
        if(*str != '-') 
            lcdPutChar(*str);
        ++str;
    }
}

void lcdCursor(bool verdict) {
    if(verdict) lcdPutCommand(0b00001111);
    else lcdPutCommand(0b00001100);
}

void lcdSetCursorXY(uint8_t x, uint8_t y) {
    lcdHome();
    if(y != 1) lcdPutCommand(0xc0);
    while(x != 1) {
        lcdPutCommand(0b00010100);
        x--;
    }
}

void lcdInit(void) {
    // Set up ports...
    DDRB = 0b00000111;
    PORTB = ~DDRB;
    DDRC = 0x0f;
    PORTC = ~DDRC;
    _delay_ms(20);
    // Set LCD to 4-bit mode...
    lcdPutCommand(0b00111100);
    // Enable 5x7 mode for chars...
    lcdPutCommand(0x28);
    // Set cursor on and to blink...
    lcdCursor(true);
    // Clear display...
    lcdClear();
    // Move the cursor to beginning of first line...
    lcdHome();
}

#endif // _LCD_LIB_C_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// TWI implementation...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _TWI_LIB_C_
#define _TWI_LIB_C_

void twiInit(void) {
    // Set SCL to 400 kHz, and we don't need need a prescaler...
    TWSR = 0x00;
    TWBR = 0x0C;
    // Enable TWI...
    TWCR = (1 << TWEN);
}

void twiStart(uint8_t addr) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    // Wait for status (until TWINT resets zero)...
    while(!(TWCR & (1 << TWINT)));
    twiWrite(addr);
}

void twiStop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void twiWrite(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)));
}

// Read a byte with acknowledgement..
uint8_t twiAckRead(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while(!(TWCR & (1<<TWINT)));
    return TWDR;
}

// Read a byte with no acknowledgement..
uint8_t twiNoAckRead(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while(!(TWCR & (1<<TWINT)));
    return TWDR;
}

uint8_t twiGetStatus(void) {
    uint8_t status;
    status = TWSR & 0xF8;
    return status;
}

#endif // _TWI_LIB_C_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Utilities implementation...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _UTIL_LIB_C_
#define _UTIL_LIB_C_

// Unsigned 16 bit integer to string...
char * u16bts(uint16_t num, char buffer[]) {
    for(int i = 4; i >= 0; i--) {
        buffer[i] = (num % 10) + '0';
        num /= 10;
    }
    // Get rid of leading zeros...
    for(int i = 0; i <= 4; i++) {
        if(buffer[i] != '0') 
            break;
        buffer[i] = '-';
    }
    return buffer;
}

#endif // _UTIL_LIB_C_

    ////////////////////////////////////
    ////////////////////////////////////
    ////////////////////////////////////
    ///       SOFTWARE SECTION       ///
    ////////////////////////////////////
    ////////////////////////////////////
    ////////////////////////////////////

#ifndef _ECE_322_FINAL_SOFTWARE_C_
#define _ECE_322_FINAL_SOFTWARE_C_

void init(void) {
    uart0_initialize(9600);
    twiInit();
    // Set up MAX's INT pin as input on D7...
    DDRD |= 0b10000000;
    PORTD &= ~(0b10000000);
    maxInit();
    lcdInit();
}

int main(void) {
    init();
    uint16_t irBuffer[SAMPLE_PACKET_SIZE];
    uint16_t redBuffer[SAMPLE_PACKET_SIZE];
    int32_t dataOut[SAMPLE_PACKET_SIZE];
    struct maxSample_t samp;
    while(true) {
        lcdClear();
        lcdPutString("Hello world!");
        // Gather SAMPLE_PACKET_SIZE samples, send it to convert data...
        for(int sample = 0; sample < SAMPLE_PACKET_SIZE; sample++) {
            while(((PIND & 0b10000000) >> 7));
            maxReadFifo(&samp.red, &samp.ir);
            irBuffer[sample] = samp.ir;
            redBuffer[sample] = samp.red;
        }
        maxConvertData(irBuffer, redBuffer, dataOut);
        char dataPoint[12];
        for(int i = 0; i < SAMPLE_PACKET_SIZE; i++) {
            sprintf(dataPoint, "%d", dataOut[i]);
            uart0_puts(dataPoint);
            uart0_putc('\n');
            _delay_ms(10);
        }
    }
    return 0;
}

#endif // _ECE_322_FINAL_SOFTWARE_C_
#endif // _ECE_322_FINAL_SOLUTION_C_

/*

==========================================
    Reference connections...
    LCD:
        RS -> B0
        RW -> B1
        E -> B2
        D4 -> C0
            ...
        D7 -> C3

    MAX:
        SDA (Breadboard C3) -> SDA (Uno)
        SCL (Breadboard C2) -> SCL (Uno)
        INT (Breadboard I1) -> D7
==========================================

*/
