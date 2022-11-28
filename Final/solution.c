////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file ECE-322/Final/solution.c
/// @brief This embedded C program operates to satisfy the requirements of ECE 322 Lab 8 (final project).
/// @authors Ian Wilkey, Andrew Kamp, Rachel Gottschalk
/// @date 11/27/2022
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _ECE_322_FINAL_SOLUTION_C_
#define _ECE_322_FINAL_SOLUTION_C_

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>

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

        // TWI methods...
        void maxInit(void);
        void maxReadFifo(uint16_t * red, uint16_t * ir);
        void maxWriteRegister(uint8_t toAddr, uint8_t data);
        void maxReadRegister(uint8_t fromAddr, uint8_t * intoData);
        void maxReset(void);
        
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
    // Wait tHD2 >=10ns (1us is OK)...
    _delay_us(1);
    // Wait tHD1 >=10ns (1us is OK)...
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
    DDRC = 0xff;
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Application...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void init(void) {
    twiInit();
    DDRD |= 0b10000000;
    PORTD &= ~(0b10000000);
    maxInit();
    lcdInit();
}

char nb[16] = { '-' };
int main(void) {
    init();
    uint16_t red, ir;
    char numBuffer[16] = { '-' };
    while(true) {
        lcdClear();
        while(((PIND & 0b10000000) >> 7));
        maxReadFifo(&red, &ir);
        lcdPutString("Red: ");
        lcdPutString(u16bts(red, nb));
        lcdSetCursorXY(1, 2);
        lcdPutString("IR: ");
        lcdPutString(u16bts(ir, nb));
        _delay_ms(50);
    }
    return 0;
}

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
==========================================

*/
