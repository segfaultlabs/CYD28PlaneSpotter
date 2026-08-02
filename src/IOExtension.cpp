#include "IOExtension.h"

bool IOExtension::begin() {
    uint8_t data[2] = {REG_MODE, 0xFF};  // all 8 IOs -> output mode
    Wire.beginTransmission(ADDR);
    Wire.write(data, sizeof(data));
    return Wire.endTransmission() == 0;
}

void IOExtension::setOutput(uint8_t pin, bool value) {
    if (value) outputState |= (1 << pin);
    else outputState &= ~(1 << pin);

    uint8_t data[2] = {REG_OUTPUT, outputState};
    Wire.beginTransmission(ADDR);
    Wire.write(data, sizeof(data));
    Wire.endTransmission();
}

void IOExtension::setBacklight(uint8_t percent) {
    // Vendor firmware clamps to 97 to avoid the backlight fully cutting off.
    if (percent > 97) percent = 97;
    uint8_t scaled = (uint8_t)(percent * (255 / 100.0f));

    uint8_t data[2] = {0x05, scaled};  // PWM register
    Wire.beginTransmission(ADDR);
    Wire.write(data, sizeof(data));
    Wire.endTransmission();
}
