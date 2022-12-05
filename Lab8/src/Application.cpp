#ifndef _LAB_8_APPLICATION_CPP_
#define _LAB_8_APPLICATION_CPP_

#include <avr/io.h>
#include <util/delay.h>

#include "libs/LiquidCrystal1602.cpp"
#include "libs/Serial.cpp"
#include "libs/I2C.cpp"
#include "libs/MAX30102.cpp"

int main(void) {

    // Use my library objects...
    Serial serial(9600);
    LiquidCrystal1602 lcd;
    I2C twi;
    MAX30102 max(&twi, 100);

    int ppgBuf[max.getPPGBufferSize()] = { 0 };
    for(;;) {
        max.heartDetection(ppgBuf);
        for(int i : ppgBuf)
            serial.println(i);
    }
    return 0;
}

#endif // _LAB_8_APPLICATION_CPP_
