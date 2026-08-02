#include "GT911_touch.h"

bool GT911_Touch::begin() {
    // Reset + address-selection sequence (GT911 datasheet): holding INT low
    // across the reset pulse selects I2C address 0x5D (vs. 0x14 if INT were
    // high). Reset itself is the IO-expander's touch-reset line, not a
    // direct GPIO on this board.
    pinMode(intPin, OUTPUT);
    digitalWrite(intPin, LOW);
    delay(1);

    io.setOutput(IOExtension::PIN_TOUCH_RST, false);
    delay(10);
    io.setOutput(IOExtension::PIN_TOUCH_RST, true);
    delay(10);

    pinMode(intPin, INPUT);
    delay(50);  // GT911 firmware boot time before first I2C access

    uint8_t id[4] = {0};
    readRegN(0x8140, id, 4);  // product ID register, ASCII e.g. "911\0"
    Serial.printf("[lcd7b] GT911 product ID: %c%c%c%c\n", id[0], id[1], id[2], id[3]);
    return id[0] != 0;
}

uint8_t GT911_Touch::readReg8(uint16_t reg) {
    uint8_t v = 0;
    readRegN(reg, &v, 1);
    return v;
}

void GT911_Touch::writeReg8(uint16_t reg, uint8_t value) {
    Wire.beginTransmission(ADDR);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    Wire.write(value);
    Wire.endTransmission();
}

void GT911_Touch::readRegN(uint16_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(ADDR);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    Wire.endTransmission(false);  // repeated start, no stop condition

    Wire.requestFrom((int)ADDR, (int)len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        buf[i] = Wire.read();
    }
}

bool GT911_Touch::touched() {
    uint8_t status = readReg8(0x814E);
    if ((status & 0x80) == 0) return false;  // no new report ready

    uint8_t pointCount = status & 0x0F;
    if (pointCount > 0) {
        uint8_t buf[7];
        readRegN(0x8150, buf, sizeof(buf));  // point 1: track_id, x_lo, x_hi, y_lo, y_hi, size_lo, size_hi
        lastX = buf[1] | ((uint16_t)buf[2] << 8);
        lastY = buf[3] | ((uint16_t)buf[4] << 8);
    }
    writeReg8(0x814E, 0x00);  // ack: tell the controller we've consumed this report
    return pointCount > 0;
}

void GT911_Touch::readData(uint16_t *x, uint16_t *y) {
    *x = lastX;
    *y = lastY;
}
