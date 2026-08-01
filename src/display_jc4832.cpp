/*
 * JC4832W535 (ESP32-S3, 480x320, AXS15231B over QSPI) bring-up.
 *
 * This is NOT a port of the CYD's 5-screen UI — that's real follow-up work
 * once this hardware pipeline is confirmed working on the physical unit.
 * This shows: WiFi/connection status, live aircraft count, and the nearest
 * aircraft's callsign/distance (using the same shared data layer the CYD
 * uses, unmodified), plus logs raw touch coordinates to Serial on tap so
 * the panel-specific calibration offsets below can be tuned against reality.
 *
 * Pins, the AXS15231B_touch driver, and the native-320x480-plus-software-
 * rotation approach are all borrowed from the sibling JC3248W535 (same
 * QSPI/AXS15231B silicon family — see byte-me404/JC3248W535_display_test)
 * since no JC4832W535-specific reference was found. A first attempt telling
 * the driver its native resolution was 480x320 directly produced a garbled
 * bottom third of the screen; matching the sibling's native-320x480 +
 * setRotation(1) pattern exactly fixed it. Expect the touch offsets
 * especially to still need further on-hardware retuning.
 */

#include "shared.h"
#include <Arduino_GFX_Library.h>
#include "AXS15231B_touch.h"

// Display (QSPI) pins — borrowed from JC3248W535-C, same silicon family
#define TFT_BL   1
#define TFT_CS   45
#define TFT_SCK  47
#define TFT_SDA0 21
#define TFT_SDA1 48
#define TFT_SDA2 40
#define TFT_SDA3 39

// Touch (I2C) pins — borrowed from JC3248W535-C
#define TOUCH_SDA  4
#define TOUCH_SCL  8
#define TOUCH_INT  3
#define TOUCH_ADDR 0x3B

// First attempt told the AXS15231B driver its native resolution was 480x320
// directly (no software rotation) — that produced a garbled bottom third of
// the screen (right hardware talking, wrong column/row addressing). The one
// proven-working reference for this chip family (the JC3248W535 sibling)
// instead keeps the driver at its native 320x480 and applies setRotation(1)
// in software to reach landscape. Matching that exactly here.
#define TFT_NATIVE_W 320
#define TFT_NATIVE_H 480
#define TFT_ROTATION 1  // landscape — logical width/height become 480x320 after this

Arduino_DataBus *gfxBus = new Arduino_ESP32QSPI(TFT_CS, TFT_SCK, TFT_SDA0, TFT_SDA1, TFT_SDA2, TFT_SDA3);
Arduino_GFX *gfxPanel = new Arduino_AXS15231B(gfxBus, GFX_NOT_DEFINED, 0, false, TFT_NATIVE_W, TFT_NATIVE_H);
Arduino_Canvas *gfx = new Arduino_Canvas(TFT_NATIVE_W, TFT_NATIVE_H, gfxPanel, 0, 0, 0);
AXS15231B_Touch touch(TOUCH_SCL, TOUCH_SDA, TOUCH_INT, TOUCH_ADDR, 0);

void displaySetup() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  if (!touch.begin()) {
    Serial.println("[jc4832] Failed to initialize touch module!");
  }
  touch.setRotation(TFT_ROTATION);
  // Placeholder offsets spanning the full *native* panel resolution — not yet
  // calibrated against the physical unit's real touch range (see file
  // header comment). Expect to replace these once raw coordinates from
  // checkTouch()'s Serial log are observed on hardware. Native (pre-rotation)
  // dimensions, matching the reference implementation's pattern.
  touch.enOffsetCorrection(true);
  touch.setOffsets(0, TFT_NATIVE_W - 1, TFT_NATIVE_W - 1, 0, TFT_NATIVE_H - 1, TFT_NATIVE_H - 1);

  if (!gfx->begin()) {
    Serial.println("[jc4832] Failed to initialize display!");
  }
  gfx->setRotation(TFT_ROTATION);
  gfx->fillScreen(BLACK);
  gfx->setCursor(20, 100);
  gfx->setTextSize(3);
  gfx->setTextColor(GREEN);
  gfx->print("CYD PLANE SPOTTER");
  gfx->flush();
  delay(1500);
}

void connectWiFiShow() {
  gfx->fillScreen(BLACK);
  gfx->setCursor(20, 100);
  gfx->setTextSize(2);
  gfx->setTextColor(WHITE);
  gfx->print("Connecting WiFi...");
  gfx->flush();
  connectWiFi();
  gfx->fillScreen(BLACK);
  gfx->flush();
}

void applyInvertColors(bool invert) {
  // Not implemented for this bring-up screen — Dark Mode is a CYD-only
  // feature for now, same as the rest of the toggle-driven UI.
  (void)invert;
}

void render() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 500) return;  // don't hammer the QSPI bus every loop() tick
  lastDraw = now;

  gfx->fillScreen(BLACK);
  gfx->setTextSize(2);

  gfx->setCursor(10, 10);
  gfx->setTextColor(WiFi.status() == WL_CONNECTED ? GREEN : RED);
  gfx->print(WiFi.status() == WL_CONNECTED ? "WiFi OK" : "WiFi ...");

  gfx->setCursor(10, 40);
  gfx->setTextColor(WHITE);
  gfx->printf("Aircraft in range: %u", blipCount);

  gfx->setCursor(10, 70);
  if (nearest.valid) {
    gfx->setTextColor(CYAN);
    gfx->printf("Nearest: %s", nearest.callsign);
    gfx->setCursor(10, 100);
    gfx->printf("%.1f km", nearest.distanceKm);
  } else {
    gfx->setTextColor(YELLOW);
    gfx->print("No aircraft in range");
  }

  gfx->flush();
}

void checkTouch() {
  if (touch.touched()) {
    uint16_t x, y;
    touch.readData(&x, &y);
    Serial.printf("[jc4832] Touch: x=%d, y=%d\n", x, y);
  }
}
