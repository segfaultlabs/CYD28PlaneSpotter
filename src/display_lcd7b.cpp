/*
 * ESP32-S3-Touch-LCD-7B (Waveshare, 1024x600, ST7701 direct RGB-parallel
 * panel + GT911 touch) bring-up.
 *
 * This is NOT a port of the CYD's 5-screen UI — that's real follow-up work
 * once this hardware pipeline is confirmed working on the physical unit
 * (same two-pass approach used for the JC4832W535). This shows: WiFi/
 * connection status, live aircraft count, and the nearest aircraft's
 * callsign/distance, using the same shared data layer the other boards use,
 * plus logs raw GT911 touch coordinates to Serial on tap.
 *
 * Every pin, timing value, and register protocol here is sourced directly
 * from Waveshare's own official example repo (github.com/waveshareteam/
 * ESP32-S3-Touch-LCD-7B, examples/Arduino/examples/06_LCD and 08_TOUCH) —
 * not guessed. Two things confirmed from that reference that shape this
 * file:
 *
 *  1. The ST7701 panel needs NO vendor init sequence at all — Waveshare's
 *     own rgb_lcd_port.c calls esp_lcd_new_rgb_panel()+esp_lcd_panel_init()
 *     directly with no ST7701 register writes, no reset pin, no disp pin.
 *     It comes up in RGB passthrough mode by itself. Arduino_GFX's
 *     Arduino_RGB_Display supports exactly this "bare" mode (bus/rst/
 *     init_operations all omitted) — used below, already present in the
 *     Arduino_GFX 1.4.5 already vendored for the JC4832W535.
 *
 *  2. Touch reset and the LCD backlight are NOT direct GPIOs on this board
 *     — they're behind a small I2C GPIO-expander (IOExtension, addr 0x24).
 *     GT911 itself is read directly over I2C using its public register map
 *     (GT911_Touch) rather than the vendor's gt911.cpp, which is just the
 *     generic ESP-IDF esp_lcd_touch_gt911 component — far more machinery
 *     (sleep modes, interrupt callbacks) than this project needs.
 */

#include "shared.h"
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include "IOExtension.h"
#include "GT911_touch.h"

// I2C bus shared by the IO-expander and GT911 touch
#define I2C_SDA 8
#define I2C_SCL 9
#define TOUCH_INT 4

// RGB parallel bus — pins/timings from Waveshare's rgb_lcd_port.h, verbatim
#define PIN_DE 5
#define PIN_VSYNC 3
#define PIN_HSYNC 46
#define PIN_PCLK 7

#define PIN_R0 1   // R3
#define PIN_R1 2   // R4
#define PIN_R2 42  // R5
#define PIN_R3 41  // R6
#define PIN_R4 40  // R7

#define PIN_G0 39  // G2
#define PIN_G1 0   // G3
#define PIN_G2 45  // G4
#define PIN_G3 48  // G5
#define PIN_G4 47  // G6
#define PIN_G5 21  // G7

#define PIN_B0 14  // B3
#define PIN_B1 38  // B4
#define PIN_B2 18  // B5
#define PIN_B3 17  // B6
#define PIN_B4 10  // B7

#define LCD_W 1024
#define LCD_H 600

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    PIN_DE, PIN_VSYNC, PIN_HSYNC, PIN_PCLK,
    PIN_R0, PIN_R1, PIN_R2, PIN_R3, PIN_R4,
    PIN_G0, PIN_G1, PIN_G2, PIN_G3, PIN_G4, PIN_G5,
    PIN_B0, PIN_B1, PIN_B2, PIN_B3, PIN_B4,
    // hsync/vsync_polarity=0: Arduino_ESP32RGBPanel::getFrameBuffer() writes
    // hsync_polarity/vsync_polarity directly into the raw LCD_CAM.lcd_ctrl2
    // idle-pol registers *after* esp_lcd_new_rgb_panel()/panel_init() already
    // ran — it overrides whatever the hsync_idle_low/vsync_idle_low struct
    // flags produced, so those flags (and matching the vendor's ESP-IDF
    // struct defaults) are irrelevant here. 0/0 matches two independent
    // working Arduino_GFX/LovyanGFX configs for this exact pin-compatible
    // Waveshare board family (different panel sizes, same GPIO layout).
    0 /* hsync_polarity */, 48 /* hsync_front_porch */, 162 /* hsync_pulse_width */, 152 /* hsync_back_porch */,
    0 /* vsync_polarity */, 3 /* vsync_front_porch */, 45 /* vsync_pulse_width */, 13 /* vsync_back_porch */,
    // 30MHz (the vendor's ESP-IDF value) exceeds the sustainable pixel clock
    // for Octal PSRAM @ 80MHz on a bounce-buffer-less RGB panel setup (~22MHz
    // ceiling) — Arduino_ESP32RGBPanel doesn't configure a bounce buffer the
    // way the vendor's own esp_lcd example does, so the framebuffer is read
    // from PSRAM directly by DMA at the full pixel rate. That mismatch reads
    // as a rolling/scrambled image, not a clean failure. 16MHz matches the
    // confirmed-working config for a pin-compatible sibling Waveshare board;
    // the library's own internal default for non-Quad-PSRAM boards is 12MHz.
    1 /* pclk_active_neg */, 16000000 /* pclk_hz */, false /* useBigEndian */);

// Bare mode: no companion bus, no reset pin, no vendor init sequence — the
// panel self-configures into RGB passthrough (see file header). auto_flush
// means draw calls write straight into the panel's own PSRAM framebuffer.
Arduino_GFX *gfx = new Arduino_RGB_Display(LCD_W, LCD_H, rgbpanel, 0, true);

IOExtension ioExpander;
GT911_Touch touch(TOUCH_INT, ioExpander);

void displaySetup() {
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!ioExpander.begin()) {
    Serial.println("[lcd7b] Failed to initialize IO expander!");
  }
  ioExpander.setBacklight(80);

  if (!touch.begin()) {
    Serial.println("[lcd7b] Failed to initialize GT911 touch!");
  }

  // Temporary diagnostic route while bringing up touch on this board — dumps
  // recent GT911 status-register polls over HTTP so they're checkable without
  // a stable USB-serial connection (this board's native USB CDC has proven
  // flaky across resets during bring-up).
  server.on("/gt911debug", []() {
    server.send(200, "text/plain", touch.dumpDebug());
  });

  if (!gfx->begin()) {
    Serial.println("[lcd7b] Failed to initialize display!");
  }
  gfx->fillScreen(BLACK);
  gfx->setCursor(20, 100);
  gfx->setTextSize(4);
  gfx->setTextColor(GREEN);
  gfx->print("CYD PLANE SPOTTER");
  gfx->flush();
  delay(1500);
}

void connectWiFiShow() {
  gfx->fillScreen(BLACK);
  gfx->setCursor(20, 100);
  gfx->setTextSize(3);
  gfx->setTextColor(WHITE);
  gfx->print("Connecting WiFi...");
  gfx->flush();
  connectWiFi();
  gfx->fillScreen(BLACK);
  gfx->flush();
}

void applyInvertColors(bool invert) {
  // Not implemented for this bring-up screen — same deferred-to-full-port
  // treatment the JC4832W535 got in its first pass.
  (void)invert;
}

void render() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 500) return;  // don't hammer the RGB/I2C path every loop() tick
  lastDraw = now;

  // Arduino_RGB_Display's auto_flush mode writes straight into the live,
  // continuously-scanned framebuffer (no back buffer) — a fillScreen() every
  // cycle is visible as a black flash each time. Clear once, then every
  // subsequent draw uses opaque (fg,bg) text color so each redraw overwrites
  // its own old footprint in place instead of needing a full clear.
  static bool cleared = false;
  if (!cleared) {
    gfx->fillScreen(BLACK);
    cleared = true;
  }
  gfx->setTextSize(3);

  gfx->setCursor(20, 20);
  gfx->setTextColor(WiFi.status() == WL_CONNECTED ? GREEN : RED, BLACK);
  gfx->print(WiFi.status() == WL_CONNECTED ? "WiFi OK   " : "WiFi ...  ");

  gfx->setCursor(20, 70);
  gfx->setTextColor(WHITE, BLACK);
  gfx->printf("Aircraft in range: %u   ", blipCount);

  gfx->setCursor(20, 120);
  if (nearest.valid) {
    gfx->setTextColor(CYAN, BLACK);
    gfx->printf("Nearest: %s          ", nearest.callsign);
    gfx->setCursor(20, 170);
    gfx->printf("%.1f km   ", nearest.distanceKm);
  } else {
    gfx->setTextColor(YELLOW, BLACK);
    gfx->print("No aircraft in range          ");
  }

  gfx->flush();
}

void checkTouch() {
  if (touch.touched()) {
    uint16_t x, y;
    touch.readData(&x, &y);
    Serial.printf("[lcd7b] Touch: x=%d, y=%d\n", x, y);
    // Visual confirmation for bring-up — this screen has no other touch
    // reaction yet (no screen-cycling UI until the full port). Draws
    // directly into the live framebuffer; the next render() cycle's opaque
    // text redraws will cover it if it lands over existing text.
    gfx->fillCircle(x, y, 12, YELLOW);
  }
}
