#include "IOExtension.h"

// Serializes access to the shared I2C bus (IO expander + GT911 touch).
// Needed because the web server task (Core 0) can trigger setBacklight()
// via /save while the render loop (Core 1) is polling the touch controller
// — raw Wire transactions are not thread-safe. Created in begin().
SemaphoreHandle_t i2cMutex = NULL;

bool IOExtension::begin() {
    if (!i2cMutex) i2cMutex = xSemaphoreCreateMutex();
    uint8_t data[2] = {REG_MODE, 0xFF};  // all 8 IOs -> output mode
    if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
    Wire.beginTransmission(ADDR);
    Wire.write(data, sizeof(data));
    bool ok = Wire.endTransmission() == 0;
    if (i2cMutex) xSemaphoreGive(i2cMutex);
    return ok;
}

void IOExtension::setOutput(uint8_t pin, bool value) {
    if (value) outputState |= (1 << pin);
    else outputState &= ~(1 << pin);

    uint8_t data[2] = {REG_OUTPUT, outputState};
    if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
    Wire.beginTransmission(ADDR);
    Wire.write(data, sizeof(data));
    Wire.endTransmission();
    if (i2cMutex) xSemaphoreGive(i2cMutex);
}

void IOExtension::setBacklight(uint8_t percent) {
    // The PWM register is active-low, confirmed directly from Waveshare's own
    // vendor example (15_LVGL_SLIDER/15_LVGL_SLIDER.ino): "Set the PWM duty
    // cycle based on slider value (inverted: 100 = off, 0 = full brightness)"
    // / "0 = full brightness due to active-low". Our percent parameter is
    // the normal user-facing sense (100 = brightest), so it has to be
    // inverted before going out as a duty cycle -- this was missing before,
    // which is exactly why 1% looked bright and 100% looked dark.
    if (percent > 100) percent = 100;
    uint8_t duty = 100 - percent;
    // Vendor firmware clamps to 97 (i.e. never lets duty reach 100) to avoid
    // the backlight fully cutting off at the dim end.
    if (duty > 97) duty = 97;
    uint8_t scaled = (uint8_t)(duty * (255 / 100.0f));

    uint8_t data[2] = {0x05, scaled};  // PWM register
    if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
    Wire.beginTransmission(ADDR);
    Wire.write(data, sizeof(data));
    Wire.endTransmission();
    if (i2cMutex) xSemaphoreGive(i2cMutex);
}
