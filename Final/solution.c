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

    #define min(x, y) ((x < y) ? x : y)

    char * u16bts(uint16_t num, char buffer[]);
    void findPeaks(int32_t * pn_locs, int32_t * n_npks,  int32_t  * pn_x, 
        int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num);
    void peaksAboveMinHeight(int32_t * pn_locs, int32_t * n_npks,  int32_t  * pn_x, 
        int32_t n_size, int32_t n_min_height);
    void removeClosePeaks(int32_t * pn_locs, int32_t * pn_npks, int32_t * pn_x, 
        int32_t n_min_distance);
    void sortAscend(int32_t * pn_x, int32_t n_size);
    void sortDecend(int32_t * pn_x, int32_t * pn_indx, int32_t n_size);

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

     // Sampling frequency...
    #define FS 25
    #define BUFFER_SIZE (FS * 4)
    #define MA4_SIZE 4

        // uch_spo2_table is approximated as -45.060 * ratioAverage * ratioAverage + 30.354 * ratioAverage + 94.845...
        const uint8_t uch_spo2_table[184]={ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 
            99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
            100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 
            97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 
            90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 
            80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 
            66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50, 
            49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29, 
            28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 
            3, 2, 1 };

        // IR buffer...
        static int32_t an_x[BUFFER_SIZE];
        // RED buffer...
        static int32_t an_y[BUFFER_SIZE];

        // TWI methods...
        void maxInit(void);
        void maxReadFifo(uint16_t * red, uint16_t * ir);
        void maxWriteRegister(uint8_t toAddr, uint8_t data);
        void maxReadRegister(uint8_t fromAddr, uint8_t * intoData);
        void maxReset(void);

        // Application methods...
        void maxGetHeartRateAndSpO2(uint16_t * irBuf, int32_t irBufLength, uint16_t * redBuf, 
            int32_t * spo2, int8_t * spo2Valid, int32_t * heartRate, int8_t * heartRateValid);
        
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

void maxGetHeartRateAndSpO2(uint16_t * irBuf, int32_t irBufLength, uint16_t * redBuf, 
    int32_t * spo2, int8_t * spo2Valid, int32_t * heartRate, int8_t * heartRateValid) {

    uint32_t un_ir_mean, un_only_once;
    int32_t k, n_i_ratio_count;
    int32_t i, s, m, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t n_th1, n_npks, n_c_min;   
    int32_t an_ir_valley_locs[15];
    int32_t n_peak_interval_sum;

    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc; 
    int32_t n_y_dc_max, n_x_dc_max; 
    int32_t n_y_dc_max_idx, n_x_dc_max_idx; 
    int32_t an_ratio[5], n_ratio_average; 
    int32_t n_nume, n_denom ;

    // Calculate DC mean and subtract DC from ir...
    un_ir_mean = 0; 
    for(k = 0; k < irBufLength; k++) 
        un_ir_mean += irBuf[k];
    un_ir_mean = un_ir_mean / irBufLength;

    // Remove DC and invert signal so that we can use peak detector as valley detector...
    for(k = 0; k < irBufLength; k++)  
        an_x[k] = -1 * (irBuf[k] - un_ir_mean); 

    // 4 pt Moving Average...
    for(k = 0; k < BUFFER_SIZE-MA4_SIZE; k++)
        an_x[k] = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]) / (int)4;        
    
    // Calculate threshold  
    n_th1 = 0; 
    for(k = 0; k < BUFFER_SIZE; k++)
        n_th1 +=  an_x[k];
    
    n_th1 = n_th1 / (BUFFER_SIZE);

    // Min allowed...
    if(n_th1 < 30) n_th1 = 30;
    // Max allowed...
    if(n_th1 > 60) n_th1 = 60; // max allowed

    for(k = 0; k < 15; k++) 
        an_ir_valley_locs[k] = 0;
    // Since we flipped signal, we use peak detector as valley detector
    // Peak_height, peak_distance, max_num_peaks...
    findPeaks(an_ir_valley_locs, &n_npks, an_x, BUFFER_SIZE, n_th1, 4, 15);
    n_peak_interval_sum =0;
    if(n_npks >= 2) {
        for(k = 1; k < n_npks; k++) 
            n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k - 1]);
        n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);
        *heartRate = (int32_t)((FS * 60) / n_peak_interval_sum);
        *heartRateValid  = 1;
    } else { 
        // unable to calculate because # of peaks are too small
        *heartRate = -999;
        *heartRateValid = 0;
    }

    //  load raw value again for SPO2 calculation : RED(=y) and IR(=X)
    for(k = 0; k < irBufLength; k++) {
        an_x[k] = irBuf[k]; 
        an_y[k] = redBuf[k]; 
    }

    // Find precise min near an_ir_valley_locs...
    n_exact_ir_valley_locs_count = n_npks;

    // Using exact_ir_valley_locs, find ir-red DC andir-red AC for SPO2 calibration an_ratio
    // finding AC/DC maximum of raw...
    n_ratio_average = 0; 
    n_i_ratio_count = 0; 
    for(k = 0; k < 5; k++) 
        an_ratio[k] = 0;
    for(k = 0; k < n_exact_ir_valley_locs_count; k++) {
        if(an_ir_valley_locs[k] > BUFFER_SIZE) {
            // Do not use SPO2 since valley loc is out of range...
            *spo2 = -999;
            *spo2Valid = 0; 
            return;
        }
    }
    // find max between two valley locations 
    // and use an_ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2 
    for(k = 0; k < n_exact_ir_valley_locs_count - 1; k++) {

        n_y_dc_max= -16777216; 
        n_x_dc_max= -16777216; 

        if(an_ir_valley_locs[k + 1] - an_ir_valley_locs[k] > 3) {
            for(i = an_ir_valley_locs[k]; i < an_ir_valley_locs[k + 1]; i++) {
                if(an_x[i] > n_x_dc_max) {
                    n_x_dc_max =an_x[i]; 
                    n_x_dc_max_idx=i;
                }
                if(an_y[i] > n_y_dc_max) {
                    n_y_dc_max = an_y[i]; 
                    n_y_dc_max_idx = i;
                }
            }
            // Red...
            n_y_ac = (an_y[an_ir_valley_locs[k + 1]] - an_y[an_ir_valley_locs[k]]) * (n_y_dc_max_idx - an_ir_valley_locs[k]);
            n_y_ac = an_y[an_ir_valley_locs[k]] + n_y_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]); 
            // Subracting linear DC compoenents from raw...
            n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac;
            // IR...
            n_x_ac = (an_x[an_ir_valley_locs[k + 1]] - an_x[an_ir_valley_locs[k]]) * (n_x_dc_max_idx - an_ir_valley_locs[k]);
            n_x_ac = an_x[an_ir_valley_locs[k]] + n_x_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]);
            // Subracting linear DC compoenents from raw...
            n_x_ac = an_x[n_y_dc_max_idx] - n_x_ac;
            // Prepare X100 to preserve floating value...
            n_nume = (n_y_ac * n_x_dc_max) >> 7;
            n_denom = (n_x_ac * n_y_dc_max) >> 7;
            if(n_denom>0  && n_i_ratio_count <5 &&  n_nume != 0) {   
                // Formula is (n_y_ac * n_x_dc_max) / (n_x_ac * n_y_dc_max)...
                an_ratio[n_i_ratio_count] = (n_nume * 100) / n_denom;
                n_i_ratio_count++;
            }
        }
    }

    // Choose median value since PPG signal may varies from beat to beat...
    sortAscend(an_ratio, n_i_ratio_count);
    n_middle_idx = n_i_ratio_count / 2;

    // Use median?
    if(n_middle_idx > 1) n_ratio_average = (an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2;
    else n_ratio_average = an_ratio[n_middle_idx];

    if(n_ratio_average > 2 && n_ratio_average < 184){
        n_spo2_calc = uch_spo2_table[n_ratio_average];
        *spo2 = n_spo2_calc;
        // float SPO2 = -45.060 * n_ratio_average * n_ratio_average / 10000 + 30.354 * n_ratio_average / 100 + 94.845...
        *spo2Valid = 1;
    } else {
        // Do not use SPO2 since signal an_ratio is out of range
        *spo2 = -999;
        *spo2Valid = 0; 
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
        if(buffer[i] != '0') break;
        buffer[i] = '-';
    }
    return buffer;
}

void findPeaks(int32_t * pn_locs, int32_t * n_npks,  int32_t  * pn_x, 
    int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num) {
    peaksAboveMinHeight(pn_locs, n_npks, pn_x, n_size, n_min_height);
    removeClosePeaks(pn_locs, n_npks, pn_x, n_min_distance);
    *n_npks = min(*n_npks, n_max_num);
}

void peaksAboveMinHeight(int32_t * pn_locs, int32_t * n_npks,  int32_t  * pn_x, 
    int32_t n_size, int32_t n_min_height) {
    int32_t i = 1, n_width;
    *n_npks = 0;
    while(i < n_size - 1) {
        if(pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]) {
            // Find left edge of potential peaks...
            n_width = 1;
            // Find flat peaks...
            while(i + n_width < n_size && pn_x[i] == pn_x[i + n_width])
                n_width++;
            // Find right edge of peaks...
            if(pn_x[i] > pn_x[i + n_width] && (*n_npks) < 15) {
                pn_locs[(*n_npks)++] = i;    
                // For flat peaks, peak location is left edge...
                i += n_width + 1;
            } else 
                i += n_width;
        } else 
            i++;
    }
}

void removeClosePeaks(int32_t * pn_locs, int32_t * pn_npks, int32_t * pn_x, 
    int32_t n_min_distance) {
    int32_t i, j, n_old_npks, n_dist;
    
    // Order peaks from large to small...
    sortDecend(pn_x, pn_locs, *pn_npks);

    for(i = -1; i < *pn_npks; i++) {
        n_old_npks = *pn_npks;
        *pn_npks = i + 1;
        for(j = i + 1; j < n_old_npks; j++) {
            // Lag-zero peak of autocorr is at index -1...
            n_dist = pn_locs[j] - (i == -1 ? -1 : pn_locs[i]);
            if(n_dist > n_min_distance || n_dist < -n_min_distance)
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }

    // Resort indices int32_to ascending order...
    sortAscend(pn_locs, *pn_npks);
}

void sortAscend(int32_t * pn_x, int32_t n_size) {
    int32_t i, j, n_temp;

    for(i = 1; i < n_size; i++) {
        n_temp = pn_x[i];
        for(j = i; j > 0 && n_temp < pn_x[j - 1]; j--)
            pn_x[j] = pn_x[j - 1];
        pn_x[j] = n_temp;
    }

}

void sortDecend(int32_t * pn_x, int32_t * pn_indx, int32_t n_size) {
    int32_t i, j, n_temp;

    for(i = 1; i < n_size; i++) {
        n_temp = pn_indx[i];
        for(j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--)
            pn_indx[j] = pn_indx[j - 1];
        pn_indx[j] = n_temp;
    }

}

#endif // _UTIL_LIB_C_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Application...
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void init(void) {
    twiInit();
    maxReset();
    maxInit();
    lcdInit();
}

int main(void) {
    init();
    uint16_t red, ir;
    char numBuffer[16] = { '-' };
    while(true) {
        lcdClear();
        maxReadFifo(&red, &ir);
        lcdPutString(u16bts(ir, numBuffer));
        _delay_ms(100);
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
