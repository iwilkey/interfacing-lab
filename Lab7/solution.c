////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file ECE-322/Lab7/solution.c
/// @brief This embedded C program operates to satisfy the requirements of ECE 322 Lab 7.
/// @authors Ian Wilkey, Andrew Kamp, Rachel Gottschalk
/// @date 10/25/2022
///
/// @attention >>> NOTE FOR GRADER <<<
/// @attention This is a complete refactoring of Dr. Mali's libraries, 
/// @attention but all functionality is kept.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _ECE_322_LAB_7_SOLUTION_C_
#define _ECE_322_LAB_7_SOLUTION_C_

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>

#define F_CPU 16000000UL
#define xtal 16000000L

// LCD library header...
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

// MPU library header...
#ifndef _MPU6050_LIB_H_
#define _MPU6050_LIB_H_

    #define MPU6050_ADDR 0x68
    #define ACCEL_XOUT_H 0x3B
    #define ACCEL_XOUT_L 0x3C
    #define ACCEL_YOUT_H 0x3D
    #define ACCEL_YOUT_L 0x3E
    #define ACCEL_ZOUT_H 0x3F
    #define ACCEL_ZOUT_L 0x40
    #define PWR_MGMT_1 0x6B

        typedef struct {
            float x;
            float y;
            float z;
        } mpu_data_t;

        void mpu_init(void);
        void mpu_get_accel(mpu_data_t * mpu_data);

#endif // _MPU6050_LIB_H_

// TWI library...
#ifndef _TWI_LIB_H_
#define _TWI_LIB_H_

    #define DEBUG_LOG 0
    #define SUCCESS	0
    #define TW_SCL_PIN PORTC5
    #define TW_SDA_PIN PORTC4
    #define TW_SLA_W(ADDR) ((ADDR << 1) | TW_WRITE)
    #define TW_SLA_R(ADDR) ((ADDR << 1) | TW_READ)
    #define TW_READ_ACK 1
    #define TW_READ_NACK 0

        typedef uint16_t ret_code_t;
        typedef enum {
            TW_FREQ_100K,
            TW_FREQ_250K,
            TW_FREQ_400K
        } twi_freq_mode_t;

        void tw_init(twi_freq_mode_t twi_freq, bool pullup_en);
        ret_code_t tw_master_transmit(uint8_t slave_addr, uint8_t * p_data, uint8_t len, bool repeat_start);
        ret_code_t  tw_master_receive(uint8_t slave_addr, uint8_t * p_data, uint8_t len);

#endif // _TWI_LIB_H_

// Various utilities
#ifndef _UTILS_LIB_H_
#define _UTILS_LIB_H_

    #define STRING_BUFFER_SIZE 256
    #define NUMBER_CONVERSION_BUFFER_SIZE 64

        char FLOAT_BUFFER[NUMBER_CONVERSION_BUFFER_SIZE] = { '-' },
            SCREEN_BUFFER[STRING_BUFFER_SIZE] = { '-' };
        const char SCREEN_BUFFER_RESET[STRING_BUFFER_SIZE] = { '-' };

        char * numberToString(double number, int precision);
        void clearBuffer(void);
        void addToScreenBuffer(char * add);

#endif // _UTILS_LIB_H_

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
        if(c == '\n' || c == '\r' || count >= size) break;
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
/// MDP_6050 Implementation...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mpu_init(void) {
    uint8_t data[2] = { PWR_MGMT_1, 0 };
    ret_code_t error_code = tw_master_transmit(MPU6050_ADDR, data, sizeof(data), false);
}

static void mpu_get_accel_raw(mpu_data_t* mpu_data) {
    ret_code_t error_code;
    uint8_t data[6];
    data[0] = ACCEL_XOUT_H;
    error_code = tw_master_transmit(MPU6050_ADDR, data, 1, true);
    if(error_code != SUCCESS) { /* if desired, insert code for reporting the error */ }
    error_code = tw_master_receive(MPU6050_ADDR, data, sizeof(data));
    if(error_code != SUCCESS) { /* if desired, insert code for reporting the error */ }
    mpu_data->x = ((int16_t)data[0] << 8 | data[1]) / 16384.0;
    mpu_data->y = ((int16_t)data[2] << 8 | data[3]) / 16384.0;
    mpu_data->z = ((int16_t)data[4] << 8 | data[5]) / 16384.0;
}

void mpu_get_accel(mpu_data_t* mpu_data) {
    mpu_get_accel_raw(mpu_data);
    mpu_data->x = mpu_data->x * 9.81;
    mpu_data->y = mpu_data->y * 9.81;
    mpu_data->z = mpu_data->z * 9.81;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// TWI Implementation...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ret_code_t tw_start(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	while(!(TWCR & (1 << TWINT)));
	if(TW_STATUS != TW_START && TW_STATUS != TW_REP_START)
        return TW_STATUS;
	return SUCCESS;
}

static void tw_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

static ret_code_t tw_write_sla(uint8_t sla) {
	TWDR = sla;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while(!(TWCR & (1 << TWINT)));
	if(TW_STATUS != TW_MT_SLA_ACK && TW_STATUS != TW_MR_SLA_ACK)
		return TW_STATUS;
    return SUCCESS;
}

static ret_code_t tw_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while(!(TWCR & (1 << TWINT)));
	if(TW_STATUS != TW_MT_DATA_ACK)
		return TW_STATUS;
	return SUCCESS;
}

static uint8_t tw_read(bool read_ack) {
	if(read_ack) {
		TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
		while(!(TWCR & (1 << TWINT)));
		if(TW_STATUS != TW_MR_DATA_ACK)
			return TW_STATUS;
	} else {
		TWCR = (1 << TWINT) | (1 << TWEN);
		while(!(TWCR & (1 << TWINT)));
		if(TW_STATUS != TW_MR_DATA_NACK) 
            return TW_STATUS;
	}
	uint8_t data = TWDR;
	return data;
}

void tw_init(twi_freq_mode_t twi_freq_mode, bool pullup_en)
{
	DDRC |= (1 << TW_SDA_PIN) | (1 << TW_SCL_PIN);
	if(pullup_en) PORTC |= (1 << TW_SDA_PIN) | (1 << TW_SCL_PIN);
	else PORTC &= ~((1 << TW_SDA_PIN) | (1 << TW_SCL_PIN));
	DDRC &= ~((1 << TW_SDA_PIN) | (1 << TW_SCL_PIN));
	switch(twi_freq_mode) {
		case TW_FREQ_100K:
		    TWBR = 72;
		    break;
		case TW_FREQ_250K:
		    TWBR = 24;
		    break;
		case TW_FREQ_400K:
            TWBR = 12;
            break;
		default: break;
	}
}

ret_code_t tw_master_transmit(uint8_t slave_addr, uint8_t* p_data, uint8_t len, bool repeat_start) {
	ret_code_t error_code;
	error_code = tw_start();
	if(error_code != SUCCESS)
		return error_code;
	error_code = tw_write_sla(TW_SLA_W(slave_addr));
	if(error_code != SUCCESS)
		return error_code;
	for(int i = 0; i < len; ++i) {
		error_code = tw_write(p_data[i]);
		if(error_code != SUCCESS)
			return error_code;
	}
	if(!repeat_start)
		tw_stop();
	return SUCCESS;
}

ret_code_t tw_master_receive(uint8_t slave_addr, uint8_t* p_data, uint8_t len) {
	ret_code_t error_code;
	error_code = tw_start();
	if(error_code != SUCCESS)
		return error_code;
	error_code = tw_write_sla(TW_SLA_R(slave_addr));
	if(error_code != SUCCESS)
		return error_code;
	for(int i = 0; i < len - 1; ++i)
		p_data[i] = tw_read(TW_READ_ACK);
	p_data[len - 1] = tw_read(TW_READ_NACK);
	tw_stop();
	return SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// LCD Implementation...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Utilities implementation...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
char * numberToString(double number, int precision) {
    return dtostrf(number, 1, precision, FLOAT_BUFFER);
}

inline void clearBuffer(void) {
    strcpy(SCREEN_BUFFER, SCREEN_BUFFER_RESET);
}

inline void addToScreenBuffer(char * add) {
    strcat(SCREEN_BUFFER, add);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Entry point...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(void) {

    lcdInit();
    uart0_initialize(9600);
    tw_init(TW_FREQ_400K, true);
    mpu_init();
    mpu_data_t accel;

    while(true) {
        lcdClear();
		mpu_get_accel(&accel);

        clearBuffer();
        addToScreenBuffer("x:");
        addToScreenBuffer(numberToString(accel.x, 1));
        addToScreenBuffer(" y:");
        addToScreenBuffer(numberToString(accel.y, 1));
        lcdPutString(SCREEN_BUFFER);
        clearBuffer();
        lcdSetCursorXY(1, 2);
        addToScreenBuffer("z:");
        addToScreenBuffer(numberToString(accel.z, 1));
        lcdPutString(SCREEN_BUFFER);

		_delay_ms(100);
    }

    return 0;
}

#endif // _ECE_322_LAB_7_SOLUTION_C_ 
