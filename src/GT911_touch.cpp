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

// Returns the number of bytes actually read — callers must check this before
// trusting buf; a short/failed I2C transaction otherwise silently leaves the
// remainder of an uninitialized caller buffer as stack garbage.
uint8_t GT911_Touch::readRegN(uint16_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(ADDR);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    Wire.endTransmission(false);  // repeated start, no stop condition

    Wire.requestFrom((int)ADDR, (int)len);
    uint8_t n = 0;
    for (; n < len && Wire.available(); n++) {
        buf[n] = Wire.read();
    }
    return n;
}

bool GT911_Touch::touched() {
    uint8_t status = readReg8(0x814E);

    static uint32_t lastDebug = 0;
    if (millis() - lastDebug > 300) {
        debugLog[debugLogPos] = status;
        debugLogPos = (debugLogPos + 1) % DEBUG_LOG_SIZE;
        lastDebug = millis();
    }

    if ((status & 0x80) == 0) return false;  // no new report ready

    uint8_t pointCount = status & 0x0F;
    bool gotPoint = false;
    if (pointCount > 0) {
        uint8_t buf[6] = {0};
        // Point 1 layout at 0x8150: x_lo, x_hi, y_lo, y_hi, size_lo, size_hi
        // — no leading track_id byte here (that's a separate field at 0x8157,
        // outside this range). An earlier off-by-one assumption here (reading
        // buf[1..4] as if buf[0] were track_id) produced wildly out-of-range
        // "coordinates" despite the I2C read itself succeeding.
        uint8_t n = readRegN(0x8150, buf, sizeof(buf));
        if (n == sizeof(buf)) {
            lastX = buf[0] | ((uint16_t)buf[1] << 8);
            lastY = buf[2] | ((uint16_t)buf[3] << 8);
            gotPoint = true;
        }
    }
    writeReg8(0x814E, 0x00);  // ack: tell the controller we've consumed this report
    return gotPoint;
}

void GT911_Touch::readData(uint16_t *x, uint16_t *y) {
    *x = lastX;
    *y = lastY;
}

String GT911_Touch::dumpDebug() {
    String out = "Last touch coords: x=" + String(lastX) + " y=" + String(lastY) + "\n";
    out += "Recent status polls (oldest first):\n";
    for (uint8_t i = 0; i < DEBUG_LOG_SIZE; i++) {
        uint8_t idx = (debugLogPos + i) % DEBUG_LOG_SIZE;
        char buf[8];
        snprintf(buf, sizeof(buf), "0x%02X ", debugLog[idx]);
        out += buf;
    }
    out += "\n";
    return out;
}
