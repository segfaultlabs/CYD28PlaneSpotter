#ifndef GT911_Touch_H
#define GT911_Touch_H

#include <Arduino.h>
#include <Wire.h>
#include "IOExtension.h"

// Minimal GT911 driver for the ESP32-S3-Touch-LCD-7B, using the well-known
// public GT911 register map (status reg 0x814E, point data from 0x8150) —
// not the vendor's gt911.cpp, which is just the generic ESP-IDF
// esp_lcd_touch_gt911 component (sleep modes, interrupt callbacks, etc)
// this project doesn't need. Touch reset on this board runs through the I2C
// IO-expander rather than a direct GPIO (see IOExtension), so begin() takes
// a reference to an already Wire.begin()'d, already-init'd expander.
class GT911_Touch {
public:
    static const uint8_t ADDR = 0x5D;

    GT911_Touch(uint8_t int_pin, IOExtension &io) : intPin(int_pin), io(io) {}

    bool begin();
    bool touched();
    void readData(uint16_t *x, uint16_t *y);

private:
    uint8_t intPin;
    IOExtension &io;
    uint16_t lastX = 0, lastY = 0;

    uint8_t readReg8(uint16_t reg);
    void writeReg8(uint16_t reg, uint8_t value);
    void readRegN(uint16_t reg, uint8_t *buf, uint8_t len);
};

#endif
