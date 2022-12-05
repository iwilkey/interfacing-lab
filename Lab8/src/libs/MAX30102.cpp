#ifndef _MAX30102_LIB_CPP_
#define _MAX30102_LIB_CPP_

#include "I2C.cpp"
#include "Serial.cpp"
#include <stdlib.h>

#define I2C_WRITE_ADDR 0xAE
#define I2C_READ_ADDR 0xAF

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

struct maxSample_t {
    uint16_t red = 0x00;
    uint16_t ir = 0x00;
};

class MAX30102 {
    public:

        MAX30102(I2C * twi, int ppgBufferSize) {
            this->ppgBufferSize = ppgBufferSize;
            this->twi = twi;
            // INTR setting...
            writeRegister(REG_INTR_ENABLE_1, 0xc0);
            writeRegister(REG_INTR_ENABLE_2, 0x00);
            // FIFO_WR_PTR[4:0]...
            writeRegister(REG_FIFO_WR_PTR, 0x00);
            // OVF_COUNTER[4:0]...
            writeRegister(REG_OVF_COUNTER, 0x00);
            // FIFO_RD_PTR[4:0]...
            writeRegister(REG_FIFO_RD_PTR, 0x00);
            // Sample avg: 4, FIFO rollover=false, FIFO almost full: 17...
            writeRegister(REG_FIFO_CONFIG, 0x4f);
            // 0x02 for red only, 0x03 for SpO2 mode 0x07 multimode LED...
            writeRegister(REG_MODE_CONFIG, 0x03);
            // SPO2_ADC range: 4096 nA, SPO2 sample rate: 100 Hz, LED pulse width: 411uS...
            writeRegister(REG_SPO2_CONFIG, 0x27);
            // Choose value for ~ 7mA for LED1...
            writeRegister(REG_LED1_PA, 0x24);
            // Choose value for ~ 7mA for LED2...
            writeRegister(REG_LED2_PA, 0x24);
            // Choose value for ~ 25mA for pilot LED...
            writeRegister(REG_PILOT_PA, 0x7f);
            // Set up MAX's INT pin as input on D7...
            DDRD |= 0b10000000;
            PORTD &= ~(0b10000000);
        }

        ~MAX30102() {}

        bool heartDetection(int * dat) {
            uint16_t irBuffer[ppgBufferSize];
            for(int i = 0; i < ppgBufferSize; i++)
                irBuffer[i] = rawSample().ir;
            // Calculate DC mean...
            unsigned irMean = 0;
            for(int i = 0; i < ppgBufferSize; i++) 
                irMean += irBuffer[i];
            irMean /= ppgBufferSize;
            // Invert...
            for(int i = 0; i < ppgBufferSize; i++)
                dat[i] = irBuffer[i] - irMean;
            
            int npt = 16;
            // Perform 4-pt moving average...
            for(int i = 0; i < ppgBufferSize - npt; i++) {
                double avg;
                for(int ii = i; ii < i + 16; ii++)
                    avg += dat[ii];
                dat[i] = (int)(avg / npt);
            }
        }

        maxSample_t rawSample(void) {
            while(PIND & 0b10000000);
            maxSample_t sample;
            readFIFO(&sample.red, &sample.ir);
            return sample;
        }

        void setPPGBufferSize(int32_t size) {
            this->ppgBufferSize = size;
        }

        int getPPGBufferSize(void) {
            return this->ppgBufferSize;
        }

    private:

        I2C * twi;
        int ppgBufferSize;

        void readFIFO(uint16_t * red, uint16_t * ir) {

            *red = 0x00;
            *ir = 0x00;

            uint8_t intrState;
            readRegister(REG_INTR_STATUS_1, &intrState);
            readRegister(REG_INTR_STATUS_2, &intrState);

            twi->start(I2C_WRITE_ADDR);
            twi->write(REG_FIFO_DATA);
            twi->stop();
            twi->start(I2C_READ_ADDR);

            uint32_t dataBuffer;

            // Read into red...
            dataBuffer = twi->read(true);
            dataBuffer <<= 16;
            *red += dataBuffer;
            dataBuffer = twi->read(true);
            dataBuffer <<= 8;
            *red += dataBuffer;
            dataBuffer = twi->read(true);
            *red += dataBuffer;

            // Read into ir...
            dataBuffer = twi->read(true);
            dataBuffer <<= 16;
            *ir += dataBuffer;
            dataBuffer = twi->read(true);
            dataBuffer <<= 8;
            *ir += dataBuffer;
            dataBuffer = twi->read(false);
            *ir += dataBuffer;

            twi->stop();

            *red &= 0x03ffff;
            *ir &= 0x03ffff;

        }

        void writeRegister(uint8_t toAddr, uint8_t data) {
            twi->start(I2C_WRITE_ADDR);
            twi->write(toAddr);
            twi->write(data);
            twi->stop();
        }
    
        void readRegister(uint8_t fromAddr, uint8_t * intoData) {
            twi->start(I2C_WRITE_ADDR);
            twi->write(fromAddr);
            twi->stop();
            twi->start(I2C_READ_ADDR);
            *intoData = twi->read(false);
            twi->stop();
        }

        inline void reset(void) {
            writeRegister(REG_MODE_CONFIG, 0x40);
        }

};

#endif // _MAX30102_LIB_CPP_
