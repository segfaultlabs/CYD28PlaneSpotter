#ifndef IOExtension_H
#define IOExtension_H

#include <Arduino.h>
#include <Wire.h>

// Small I2C GPIO-expander on the ESP32-S3-Touch-LCD-7B (Waveshare's own
// examples call it "IO_EXTENSION"; likely a CH32V003 running expander
// firmware rather than an off-the-shelf expander IC, but the I2C register
// protocol is simple and fixed regardless). Controls the LCD backlight (PWM)
// and the GT911 touch reset line — nothing else on this board needs it.
// Protocol ported from the vendor's io_extension.h/.c (register addresses
// and behavior, not code).
class IOExtension {
public:
    static const uint8_t ADDR = 0x24;
    static const uint8_t REG_MODE = 0x02;    // bit=1 -> that IO is output
    static const uint8_t REG_OUTPUT = 0x03;  // output level per IO bit
    static const uint8_t PIN_TOUCH_RST = 1;
    static const uint8_t PIN_BACKLIGHT = 2;

    bool begin();
    void setOutput(uint8_t pin, bool value);
    void setBacklight(uint8_t percent);  // 0-100

private:
    uint8_t outputState = 0xFF;  // all pins default high, matches vendor init
};

#endif
