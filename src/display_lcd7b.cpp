/*
 * ESP32-S3-Touch-LCD-7B (Waveshare, 1024x600, ST7701 direct RGB-parallel
 * panel + GT911 touch) — full UI.
 *
 * Same screen set and consolidated-Radar reasoning as the JC4832W535's full
 * port (display_jc4832.cpp): Target Intel, Top 5, Radar, Weather & System.
 * Icons and the label collision-avoidance helper are duplicated from that
 * file rather than shared — same Arduino_GFX API family, but different
 * canvas class (Arduino_RGB_Display here vs Arduino_Canvas there) and a
 * much bigger 1024x600 layout, so little would actually be saved by sharing.
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
// panel self-configures into RGB passthrough (see file header).
//
// auto_flush=false, not true: there is only ONE framebuffer here (this
// class draws straight into the RGB panel's own live, DMA-scanned PSRAM
// buffer — confirmed by reading Arduino_RGB_Display.cpp, no shadow/back
// buffer exists). With auto_flush=true, every individual draw call
// immediately does a cache write-back, making each intermediate step of a
// multi-call redraw (clear, then rings, then blips) visible to the DMA
// scan-out as it happens — that's the flicker. DMA reads PSRAM directly
// (bypassing CPU D-cache), so as long as we DON'T force a write-back,
// DMA keeps showing the old, complete frame while we draw the new one in
// cache; a single explicit flush() at the end of a render pass (already
// called once per cycle in render()) then makes the whole new frame appear
// at once instead of incrementally.
Arduino_GFX *gfx = new Arduino_RGB_Display(LCD_W, LCD_H, rgbpanel, 0, false);

IOExtension ioExpander;
GT911_Touch touch(TOUCH_INT, ioExpander);

#define LCD7B_NUM_SCREENS 4  // Target Intel, Top 5, Radar, Weather & System

// Layout (1024x600): header 0-35px; left info column x:0-260; radar area
// x:260-1024 x y:35-600, circle centered in that region.
static const int HEADER_H = 35;
static const int RADAR_AREA_X = 260, RADAR_AREA_Y = HEADER_H;
static const int RADAR_AREA_W = LCD_W - RADAR_AREA_X, RADAR_AREA_H = LCD_H - HEADER_H;
static const int RADAR_CX = RADAR_AREA_X + RADAR_AREA_W / 2;
static const int RADAR_CY = RADAR_AREA_Y + RADAR_AREA_H / 2;
static const int RADAR_R = 260;

// Flight-path trace history (Radar screen, showTraces toggle). Keyed by
// callsign rather than blips[] array index, since that index isn't stable
// between fetch cycles — a trail must follow one aircraft, not one array
// slot. Lives here (not shared.h/data.cpp) since it's purely a rendering
// concern local to this board's Radar screen.
// 1800 samples ~= 15-30 min of history at this screen's 1-2Hz full-redraw
// sampling rate -- long enough that no realistic single viewing session
// should ever see a trail age out by hitting this cap; only true staleness
// (TRAIL_STALE_MS, aircraft actually gone) should ever clear a trail. Backed
// by PSRAM (ps_malloc, lazily allocated per slot and kept for the life of
// the program) rather than a fixed struct member -- 1800*16 bytes*20 slots
// is ~560KB, too big to want living in internal SRAM/.bss alongside
// everything else, but trivial against this board's 8MB PSRAM.
#define TRAIL_LEN 1800
#define TRAIL_SLOTS MAX_BLIPS
#define TRAIL_STALE_MS 15000  // drop a trail if its aircraft hasn't been seen in this long
struct TrailHistory {
  char callsign[10] = {0};
  double *lat = nullptr, *lon = nullptr;  // ps_malloc'd on first use, kept across resets
  uint16_t count = 0;  // TRAIL_LEN > 255, so this can't be uint8_t
  uint32_t lastSeenMs = 0;
  // Clears the trail's identity/content but keeps its PSRAM buffer
  // allocated for reuse -- a plain `t = TrailHistory()` would instead null
  // out lat/lon and leak the previous allocation.
  void reset() { callsign[0] = 0; count = 0; lastSeenMs = 0; }
  void ensureBuf() {
    if (!lat) { lat = (double *)ps_malloc(sizeof(double) * TRAIL_LEN); lon = (double *)ps_malloc(sizeof(double) * TRAIL_LEN); }
  }
};
static TrailHistory trails[TRAIL_SLOTS];

bool timeReady() { return time(nullptr) > 1700000000; }
void fmtClock(char* buf, size_t n, bool withSecs) {
  if (!timeReady()) { strncpy(buf, withSecs ? "--:--:--" : "--:--", n); return; }
  time_t t = time(nullptr); struct tm lt; localtime_r(&t, &lt);
  strftime(buf, n, withSecs ? "%H:%M:%S" : "%H:%M", &lt);
}

void drawHeader(const char* title) {
  char left[40];
  snprintf(left, sizeof(left), "%s [%d/%d]", title, screen + 1, LCD7B_NUM_SCREENS);
  gfx->setTextSize(2);
  gfx->setTextColor(LIGHTGREY, BLACK);
  gfx->setCursor(15, 10);
  gfx->print(left);

  char t[10]; fmtClock(t, sizeof(t), true);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(t, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(1010 - (int)tw, 10);
  gfx->print(t);

  gfx->drawLine(0, HEADER_H, LCD_W, HEADER_H, DARKGREY);
}

void drawArrow(int cx, int cy, int r, double angleDeg, uint16_t color) {
  double a = deg2rad(angleDeg);
  int tx = cx + (int)(sin(a) * r), ty = cy - (int)(cos(a) * r);
  int bx = cx - (int)(sin(a) * r), by = cy + (int)(cos(a) * r);
  gfx->drawLine(bx, by, tx, ty, color);
  double left = a + deg2rad(150), right = a - deg2rad(150);
  gfx->drawLine(tx, ty, tx + (int)(sin(left) * (r/2)), ty - (int)(cos(left) * (r/2)), color);
  gfx->drawLine(tx, ty, tx + (int)(sin(right)* (r/2)), ty - (int)(cos(right)* (r/2)), color);
}

// Small heading-oriented plane glyph (fuselage + wings + tailfin) for radar blips.
void drawPlaneIcon(int cx, int cy, double headingDeg, uint16_t color) {
  double h = deg2rad(headingDeg);
  double sh = sin(h), ch = cos(h);
  int noseX = cx + (int)(sh * 5),  noseY = cy - (int)(ch * 5);
  int tailX = cx - (int)(sh * 4),  tailY = cy + (int)(ch * 4);
  gfx->drawLine(tailX, tailY, noseX, noseY, color);
  int wingCx = cx + (int)(sh * 1), wingCy = cy - (int)(ch * 1);
  double pw = h + deg2rad(90);
  int wLx = wingCx + (int)(sin(pw) * 4), wLy = wingCy - (int)(cos(pw) * 4);
  int wRx = wingCx - (int)(sin(pw) * 4), wRy = wingCy + (int)(cos(pw) * 4);
  gfx->drawLine(wLx, wLy, wRx, wRy, color);
  int fLx = tailX + (int)(sin(pw) * 2), fLy = tailY - (int)(cos(pw) * 2);
  int fRx = tailX - (int)(sin(pw) * 2), fRy = tailY + (int)(cos(pw) * 2);
  gfx->drawLine(fLx, fLy, fRx, fRy, color);
}

// Rotor-cross glyph for rotorcraft (ADS-B category "A7").
void drawHelicopterIcon(int cx, int cy, double headingDeg, uint16_t color) {
  double h = deg2rad(headingDeg);
  int tailX = cx - (int)(sin(h) * 5), tailY = cy + (int)(cos(h) * 5);
  gfx->drawLine(cx, cy, tailX, tailY, color);
  gfx->drawLine(cx - 4, cy, cx + 4, cy, color);
  gfx->drawLine(cx, cy - 4, cx, cy + 4, color);
  gfx->fillCircle(cx, cy, 2, color);
}

// Small cloud glyph, top-left anchored, ~20px wide x 10px tall
void drawCloudShape(int x, int y, uint16_t color) {
  gfx->fillCircle(x + 5, y + 6, 4, color);
  gfx->fillCircle(x + 10, y + 4, 5, color);
  gfx->fillCircle(x + 15, y + 6, 4, color);
  gfx->fillRect(x + 3, y + 6, 14, 5, color);
}

// Maps an Open-Meteo WMO weather_code to a compact vector icon, top-left
// anchored in a ~20x18px box.
void drawWeatherIcon(int x, int y, int code) {
  if (code == 0) {
    int cx = x + 9, cy = y + 8;
    gfx->fillCircle(cx, cy, 5, YELLOW);
    for (int a = 0; a < 360; a += 45) {
      double r = deg2rad(a);
      gfx->drawLine(cx + (int)(sin(r) * 7), cy - (int)(cos(r) * 7),
                    cx + (int)(sin(r) * 10), cy - (int)(cos(r) * 10), YELLOW);
    }
  } else if (code >= 1 && code <= 3) {
    int cx = x + 5, cy = y + 4;
    gfx->fillCircle(cx, cy, 4, YELLOW);
    drawCloudShape(x + 2, y + 5, LIGHTGREY);
  } else if (code == 45 || code == 48) {
    for (int i = 0; i < 3; i++)
      gfx->drawFastHLine(x, y + 5 + i * 4, 18, LIGHTGREY);
  } else if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82)) {
    drawCloudShape(x, y, LIGHTGREY);
    for (int i = 0; i < 3; i++)
      gfx->drawLine(x + 4 + i * 5, y + 12, x + 2 + i * 5, y + 17, CYAN);
  } else if ((code >= 71 && code <= 77) || code == 85 || code == 86) {
    drawCloudShape(x, y, LIGHTGREY);
    for (int i = 0; i < 3; i++) {
      int sx = x + 4 + i * 5, sy = y + 15;
      gfx->drawLine(sx - 2, sy, sx + 2, sy, WHITE);
      gfx->drawLine(sx, sy - 2, sx, sy + 2, WHITE);
    }
  } else if (code >= 95) {
    drawCloudShape(x, y, DARKGREY);
    gfx->drawLine(x + 10, y + 11, x + 7, y + 16, YELLOW);
    gfx->drawLine(x + 7, y + 16, x + 11, y + 16, YELLOW);
    gfx->drawLine(x + 11, y + 16, x + 8, y + 21, YELLOW);
  } else {
    drawCloudShape(x, y, LIGHTGREY);
  }
}

// Places a multi-line blip label near (bx,by), avoiding overlap with every
// label already placed this frame. All label lines use textSize(2); the
// caller must set that before calling.
struct LabelRect { int x, y, w, h; };

bool placeLabel(LabelRect *placed, int &placedCount, int maxPlaced,
                 int bx, int by, char lines[][12], int lineCount,
                 int boundX, int boundY, int boundW, int boundH) {
  if (lineCount == 0) return false;

  int w = 0, h = 0;
  int lineY[9];
  int16_t x1, y1; uint16_t lw, lh;
  for (int i = 0; i < lineCount; i++) {
    gfx->getTextBounds(lines[i], 0, 0, &x1, &y1, &lw, &lh);
    if ((int)lw > w) w = (int)lw;
    lineY[i] = h;
    h += (int)lh + 1;
  }
  w += 3;

  const int dx[6] = { 6, -6 - w, 6, -6 - w, 6, -6 - w };
  const int dy[6] = { -6, -6, 10, -10 - h, 16, 16 };

  for (int attempt = 0; attempt < 6; attempt++) {
    int lx = bx + dx[attempt];
    int ly = by + dy[attempt];
    if (lx < boundX) lx = boundX;
    if (lx + w > boundX + boundW) lx = boundX + boundW - w;
    if (ly < boundY) ly = boundY;
    if (ly + h > boundY + boundH) ly = boundY + boundH - h;
    if (lx < boundX || ly < boundY) continue;  // label too big for the area at all

    LabelRect cand = { lx, ly, w, h };
    bool overlap = false;
    for (int i = 0; i < placedCount; i++) {
      LabelRect &r = placed[i];
      if (cand.x < r.x + r.w && cand.x + cand.w > r.x &&
          cand.y < r.y + r.h && cand.y + cand.h > r.y) {
        overlap = true;
        break;
      }
    }
    if (!overlap) {
      gfx->setTextColor(WHITE, BLACK);
      for (int li = 0; li < lineCount; li++) {
        gfx->setCursor(lx, ly + lineY[li]);
        gfx->print(lines[li]);
      }
      if (placedCount < maxPlaced) placed[placedCount++] = cand;
      return true;
    }
  }
  return false;
}

void screenTargetIntel() {
  drawHeader("TARGET INTEL");

  if (!nearest.valid) {
    gfx->setTextSize(4);
    gfx->setTextColor(RED, BLACK);
    gfx->setCursor(80, 220);
    gfx->print("NO TARGET IN RANGE");
    return;
  }

  gfx->setTextSize(5);
  gfx->setTextColor(GREEN, BLACK);
  gfx->setCursor(30, 60);
  gfx->print(nearest.callsign);

  gfx->setTextSize(2);
  gfx->setTextColor(CYAN, BLACK);
  gfx->setCursor(350, 75);
  if (nearest.hasRoute) gfx->printf("Route: %s -> %s          ", nearest.dep, nearest.arr);
  else gfx->print("Route: Unknown          ");

  gfx->setTextSize(3);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(30, 160);
  gfx->printf("Type: %s          ", nearest.country);

  gfx->setCursor(30, 210);
  gfx->printf("Distance: %.1f km (%s)   ", nearest.distanceKm, compass(nearest.bearingDeg));

  float altFeet = nearest.altitudeM * 3.28084f;
  gfx->setCursor(30, 260);
  if (nearest.onGround) gfx->print("Altitude: On Ground          ");
  else gfx->printf("Alt: %.0f ft (FL%03.0f)      ", altFeet, altFeet / 100.0f);

  gfx->setCursor(30, 310);
  gfx->printf("Speed: %.0f km/h      ", nearest.velocityMs * 3.6);

  int acx = 780, acy = 220, ar = 70;
  // Arrow is vector lines, not opaque text — needs an explicit clear before
  // each redraw or the old heading's lines remain visible alongside the new.
  gfx->fillRect(acx - ar - 15, acy - ar - 15, (ar + 15) * 2, (ar + 15) * 2 + 60, BLACK);
  drawArrow(acx, acy, ar, nearest.trackDeg, CYAN);
  gfx->setTextSize(3);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(acx - 80, acy + ar + 25);
  gfx->printf("HDG %03.0f", nearest.trackDeg);
}

void screenTop5() {
  drawHeader("TOP 5 IN RANGE");

  if (top5Count == 0) {
    gfx->setTextSize(4);
    gfx->setTextColor(RED, BLACK);
    gfx->setCursor(300, 220);
    gfx->print("NO TARGETS");
    return;
  }

  gfx->setTextSize(2);
  gfx->setTextColor(CYAN, BLACK);
  gfx->setCursor(30, 55);
  gfx->print("FLIGHT");
  gfx->setCursor(300, 55);
  gfx->print("ROUTE");
  gfx->setCursor(560, 55);
  gfx->print("ALTITUDE");
  gfx->drawLine(20, 85, 1000, 85, DARKGREY);

  int y = 110;
  gfx->setTextSize(3);
  for (int i = 0; i < top5Count; i++) {
    // Fields here can change length between redraws (a shorter callsign
    // replacing a longer one when the underlying top5[] list refreshes,
    // while `screen` itself hasn't changed and so no full clear happens) —
    // trailing padding on each covers that, same reasoning as the arrow
    // clear above.
    gfx->setTextColor(WHITE, BLACK);
    gfx->setCursor(30, y);
    gfx->printf("%-11s", top5[i].callsign);

    char rte[16];
    if (top5[i].hasRoute) snprintf(rte, sizeof(rte), "%s->%s", top5[i].dep, top5[i].arr);
    else strcpy(rte, "N/A");
    gfx->setCursor(300, y);
    gfx->printf("%-12s", rte);

    gfx->setCursor(560, y);
    if (top5[i].onGround) {
      gfx->print("Ground              ");
    } else {
      float ft = top5[i].altitudeM * 3.28084f;
      gfx->printf("%.0f ft (FL%03.0f)      ", ft, ft / 100.0f);
    }
    y += 80;
  }
}

// "-" / "+" button hit-rects, shared between the draw code below and checkTouch().
static const int RNG_BTN_X = 30, RNG_BTN_W = 180, RNG_BTN_H = 70;
static const int RNG_PLUS_Y = 380, RNG_MINUS_Y = 470;

void screenRadar() {
  drawHeader("RADAR");

  double hLat, hLon; float rMax;
  bool showCS, showAir, showSpd, showFlt, showRte, showRg, showSq, showVr, showTy;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; rMax = radarMaxKm;
  showCS = showCallsign; showAir = showAirline; showSpd = showSpeed; showFlt = showFL; showRte = showRoute;
  showRg = showReg; showSq = showSquawk; showVr = showVRate; showTy = showType;
  if (configMutex) xSemaphoreGive(configMutex);

  // --- Left info column ---
  // Every field here can change width between redraws (contact count,
  // distance, weather) without a full-screen clear (screen index hasn't
  // changed) — trailing padding covers that, no back buffer to hide it.
  gfx->setTextSize(3);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(20, 55);
  gfx->printf("Contacts: %u   ", blipCount);

  gfx->setTextColor(CYAN, BLACK);
  gfx->setCursor(20, 100);
  if (nearest.valid) {
    gfx->printf("%-11s", nearest.callsign);
    gfx->setCursor(20, 140);
    gfx->printf("%.1f km   ", nearest.distanceKm);
  } else {
    gfx->print("No TGT          ");
    gfx->setCursor(20, 140);
    gfx->print("               ");
  }

  gfx->setTextSize(2);
  if (weather.valid) {
    drawWeatherIcon(20, 190, weather.code);
    gfx->setTextColor(CYAN, BLACK);
    gfx->setCursor(55, 194);
    gfx->printf("%.1fC   ", weather.tempC);
    gfx->setCursor(20, 225);
    gfx->printf("Wind %.1f km/h   ", weather.windKmh);
  } else {
    gfx->setTextColor(CYAN, BLACK);
    gfx->setCursor(20, 225);
    gfx->print("Weather: --          ");
  }

  gfx->setTextSize(3);
  gfx->setTextColor(CYAN, BLACK);
  gfx->setCursor(20, 320);
  gfx->printf("RNG %d km   ", (int)rMax);

  gfx->fillRoundRect(RNG_BTN_X, RNG_PLUS_Y, RNG_BTN_W, RNG_BTN_H, 6, DARKGREY);
  gfx->setTextColor(WHITE, DARKGREY);
  gfx->setCursor(RNG_BTN_X + 75, RNG_PLUS_Y + 18);
  gfx->print("+");

  gfx->fillRoundRect(RNG_BTN_X, RNG_MINUS_Y, RNG_BTN_W, RNG_BTN_H, 6, DARKGREY);
  gfx->setCursor(RNG_BTN_X + 75, RNG_MINUS_Y + 18);
  gfx->print("-");

  // --- Radar circle ---
  // Unlike the CYD/JC4832W535 (which redraw an offscreen sprite/canvas and
  // blit it in one go via a real back buffer), this board's Arduino_GFX RGB
  // panel support has no double-buffering at all on this project's pinned
  // ESP32 Arduino core (2.0.14) — its bundled ESP-IDF predates the
  // num_fbs/bounce_buffer fields the vendor's own reference code relies on
  // for tear-free updates, and upgrading the core would break Arduino_GFX/
  // AXS15231B compatibility for the other two boards. So every draw here
  // lands directly on the live, continuously-scanned framebuffer, and any
  // full-area clear+redraw is visible mid-draw.
  //
  // Given that constraint, the sweep line — which needs to move every
  // frame for smooth motion — is erased and redrawn as a single thin line
  // (tiny footprint) on every call. The much heavier full redraw (clear +
  // rings + compass + blips + labels + trails) only happens every ~600ms,
  // which cuts how often the large-area flicker is visible without
  // sacrificing sweep smoothness.
  const int cx = RADAR_CX, cy = RADAR_CY, R = RADAR_R;
  float sweepDeg = fmodf(millis() / 15.0f, 360.0f);
  double sw = deg2rad(sweepDeg);
  int sweepEndX = cx + (int)(sin(sw) * R), sweepEndY = cy - (int)(cos(sw) * R);

  static uint32_t lastFullDraw = 0;
  static int prevSweepX = cx, prevSweepY = cy - R;
  static bool havePrevSweep = false;
  uint32_t nowMs = millis();
  bool doFull = (nowMs - lastFullDraw >= 600);

  if (doFull) {
    lastFullDraw = nowMs;
    gfx->fillRect(RADAR_AREA_X, RADAR_AREA_Y, RADAR_AREA_W, RADAR_AREA_H, BLACK);
    gfx->drawCircle(cx, cy, R, DARKGREY);
    gfx->drawCircle(cx, cy, R * 2 / 3, DARKGREY);
    gfx->drawCircle(cx, cy, R / 3, DARKGREY);
    gfx->drawLine(cx - R, cy, cx + R, cy, DARKGREY);
    gfx->drawLine(cx, cy - R, cx, cy + R, DARKGREY);

    // Compass — matches the same north-up, clockwise convention already
    // used for blip placement (bx = cx + sin(brg)*r, by = cy - cos(brg)*r,
    // brg 0=N/90=E/180=S/270=W), so these letters are correct relative to
    // where blips actually land, not just decorative.
    gfx->setTextSize(2);
    gfx->setTextColor(LIGHTGREY, BLACK);
    int16_t lx1, ly1; uint16_t lw, lh;
    gfx->getTextBounds("N", 0, 0, &lx1, &ly1, &lw, &lh);
    gfx->setCursor(cx - lw / 2, cy - R - lh - 6); gfx->print("N");
    gfx->setCursor(cx - lw / 2, cy + R + 6); gfx->print("S");
    gfx->getTextBounds("E", 0, 0, &lx1, &ly1, &lw, &lh);
    gfx->setCursor(cx + R + 8, cy - lh / 2); gfx->print("E");
    gfx->getTextBounds("W", 0, 0, &lx1, &ly1, &lw, &lh);
    gfx->setCursor(cx - R - 8 - lw, cy - lh / 2); gfx->print("W");

    float elapsed = (millis() - lastDataMs) / 1000.0f;

    // --- Flight path traces: per-aircraft position history, keyed by
    // callsign since blips[] array indices aren't stable between fetch
    // cycles. Sampled once per full-redraw (~600ms) rather than every
    // frame -- dense enough to show a path, sparse enough to stay cheap.
    // Entries not seen in TRAIL_STALE_MS are dropped, so a trail vanishes
    // once its aircraft is no longer tracked (out of range / off screen).
    if (showTraces) {
      for (uint8_t i = 0; i < blipCount; i++) {
        if (!blips[i].callsign[0]) continue;
        int slot = -1, oldest = 0;
        for (uint8_t s = 0; s < TRAIL_SLOTS; s++) {
          if (trails[s].callsign[0] && strcmp(trails[s].callsign, blips[i].callsign) == 0) { slot = s; break; }
          if (!trails[s].callsign[0] || trails[s].lastSeenMs < trails[oldest].lastSeenMs) oldest = s;
        }
        if (slot < 0) {
          slot = oldest;
          trails[slot].reset();
          strncpy(trails[slot].callsign, blips[i].callsign, 9); trails[slot].callsign[9] = '\0';
        }
        TrailHistory &t = trails[slot];
        t.ensureBuf();
        if (t.count < TRAIL_LEN) {
          t.lat[t.count] = blips[i].lat; t.lon[t.count] = blips[i].lon; t.count++;
        } else {
          for (uint16_t j = 1; j < TRAIL_LEN; j++) { t.lat[j-1] = t.lat[j]; t.lon[j-1] = t.lon[j]; }
          t.lat[TRAIL_LEN-1] = blips[i].lat; t.lon[TRAIL_LEN-1] = blips[i].lon;
        }
        t.lastSeenMs = nowMs;
      }
      for (uint8_t s = 0; s < TRAIL_SLOTS; s++) {
        TrailHistory &t = trails[s];
        if (!t.callsign[0]) continue;
        if (nowMs - t.lastSeenMs > TRAIL_STALE_MS) { t.reset(); continue; }
        // Connected polyline, not separate dots -- a trail of isolated 2-3px
        // dots is nearly indistinguishable from noise at this scale (typical
        // sample-to-sample movement is only a few px), and doesn't read as
        // "a path" the way a real radar trace does. YELLOW, not DARKGREY:
        // the three range rings are DARKGREY, so anything drawn in that same
        // color blends straight into them.
        int prevPx = 0, prevPy = 0; bool havePrev = false;
        for (uint16_t j = 0; j < t.count; j++) {
          double dist = haversineKm(hLat, hLon, t.lat[j], t.lon[j]);
          float fr = (float)(dist / rMax);
          if (fr > 1) { havePrev = false; continue; }
          int rr = (int)(fr * R);
          double brg = bearingDeg(hLat, hLon, t.lat[j], t.lon[j]);
          int px = cx + (int)(sin(deg2rad(brg)) * rr);
          int py = cy - (int)(cos(deg2rad(brg)) * rr);
          if (havePrev) gfx->drawLine(prevPx, prevPy, px, py, YELLOW);
          prevPx = px; prevPy = py; havePrev = true;
        }
      }
    }

    gfx->setTextSize(2);

    LabelRect placed[MAX_BLIPS];
    int placedCount = 0;

    for (uint8_t i = 0; i < blipCount; i++) {
      double la = blips[i].lat, lo = blips[i].lon;
      if (blips[i].speedMs > 0 && elapsed > 0)
        projectLatLon(blips[i].lat, blips[i].lon, blips[i].track, blips[i].speedMs * elapsed, la, lo);

      double dist = haversineKm(hLat, hLon, la, lo);
      float fr = (float)(dist / rMax);
      if (fr > 1) continue;

      int rr = (int)(fr * R);
      double brg = bearingDeg(hLat, hLon, la, lo);
      int bx = cx + (int)(sin(deg2rad(brg)) * rr);
      int by = cy - (int)(cos(deg2rad(brg)) * rr);

      float behind = fmodf(sweepDeg - (float)brg + 360.0f, 360.0f);
      uint16_t blipColor = (behind < 30) ? YELLOW : GREEN;
      if (strcmp(blips[i].category, "A7") == 0)
        drawHelicopterIcon(bx, by, blips[i].track, blipColor);
      else
        drawPlaneIcon(bx, by, blips[i].track, blipColor);

      {
        char lines[9][12];
        int lineCount = 0;
        bool hasCallsign = blips[i].callsign[0] && strcmp(blips[i].callsign, "(no id)") != 0;

        if (showCS && hasCallsign) {
          strncpy(lines[lineCount], blips[i].callsign, 11); lines[lineCount][11] = '\0'; lineCount++;
        }
        if (showFlt) {
          float altFt = blips[i].altitudeM * 3.28084f;
          snprintf(lines[lineCount], 12, "FL%03.0f", altFt / 100.0f); lineCount++;
        }
        if (showRte && blips[i].hasRoute) {
          snprintf(lines[lineCount], 12, "%s>%s", blips[i].dep, blips[i].arr); lineCount++;
        }
        if (showAir) {
          const char* airline = blips[i].airline[0] ? blips[i].airline : lookupAirline(blips[i].callsign);
          if (airline) {
            strncpy(lines[lineCount], airline, 10); lines[lineCount][10] = '\0'; lineCount++;
          }
        }
        if (showRg && blips[i].reg[0]) {
          strncpy(lines[lineCount], blips[i].reg, 11); lines[lineCount][11] = '\0'; lineCount++;
        }
        if (showSq && blips[i].squawk[0]) {
          snprintf(lines[lineCount], 12, "SQ%s", blips[i].squawk); lineCount++;
        }
        if (showVr) {
          snprintf(lines[lineCount], 12, "%+dfpm", blips[i].vrateFpm); lineCount++;
        }
        if (showTy && blips[i].typeCode[0]) {
          strncpy(lines[lineCount], blips[i].typeCode, 11); lines[lineCount][11] = '\0'; lineCount++;
        }
        if (showSpd) {
          float speedKt = blips[i].speedMs * 1.94384f;
          snprintf(lines[lineCount], 12, "%ukn", (unsigned int)(speedKt + 0.5f)); lineCount++;
        }

        placeLabel(placed, placedCount, MAX_BLIPS, bx, by, lines, lineCount,
                   RADAR_AREA_X, RADAR_AREA_Y, RADAR_AREA_W, RADAR_AREA_H);
      }
    }

    havePrevSweep = false;  // rings/blips just got redrawn fresh; no stale sweep line to erase
  } else if (havePrevSweep) {
    gfx->drawLine(cx, cy, prevSweepX, prevSweepY, BLACK);
  }

  gfx->drawLine(cx, cy, sweepEndX, sweepEndY, GREEN);
  prevSweepX = sweepEndX; prevSweepY = sweepEndY;
  havePrevSweep = true;
}

void screenWeatherSystem() {
  drawHeader("WEATHER & SYSTEM");
  gfx->drawLine(20, 250, 1000, 250, DARKGREY);

  if (!weather.valid) {
    gfx->setTextSize(4);
    gfx->setTextColor(RED, BLACK);
    gfx->setCursor(60, 120);
    gfx->print("NO WX DATA");
  } else {
    gfx->setTextSize(4);
    gfx->setTextColor(YELLOW, BLACK);
    gfx->setCursor(60, 100);
    gfx->printf("%.1f C   ", weather.tempC);

    gfx->setTextSize(3);
    gfx->setTextColor(CYAN, BLACK);
    gfx->setCursor(500, 100);
    gfx->printf("Humidity: %d%%   ", weather.humidity);
    gfx->setCursor(500, 150);
    gfx->printf("Wind: %.0f km/h   ", weather.windKmh);
  }

  gfx->setTextSize(3);
  gfx->setTextColor(WHITE, BLACK);

  uint32_t up = millis() / 1000;
  gfx->setCursor(60, 300);
  gfx->printf("Uptime: %02lu:%02lu:%02lu   ", up / 3600, (up % 3600) / 60, up % 60);

  gfx->setCursor(500, 300);
  gfx->printf("WiFi RSSI: %d dBm   ", WiFi.RSSI());

  gfx->setCursor(60, 360);
  gfx->printf("RAM Free: %d KB   ", ESP.getFreeHeap() / 1024);

  gfx->setCursor(500, 360);
  gfx->printf("PSRAM Free: %d KB   ", ESP.getFreePsram() / 1024);

  gfx->setCursor(60, 420);
  gfx->printf("API Req: %lu OK / %lu FAIL   ", stats.requestsOk, stats.requestsFail);
}

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
  // Not implemented — this is a "bare" RGB panel with no command interface
  // to the ST7701 at all (see file header), so there's no hardware color
  // invert to call the way display_cyd.cpp/display_jc4832.cpp do. A
  // software palette flip would mean touching every draw call; not worth it
  // for a nice-to-have. Dark Mode stays CYD/JC4832W535-only.
  (void)invert;
}

void applyBrightness(uint8_t percent) {
  ioExpander.setBacklight(percent);
}

void render() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  // Radar needs a much shorter interval for a smooth sweep animation (the
  // static info screens don't) — 300ms made the sweep visibly jump ~20deg
  // per frame. 120ms is a middle ground: smoother without redrawing the
  // radar's ~764x565 region (fillRect + rings + blips every cycle) often
  // enough to risk the PSRAM/DMA bandwidth issue that caused the original
  // rolling-image bug at a too-high pixel clock.
  uint32_t interval = (screen % LCD7B_NUM_SCREENS == 2) ? 120 : 400;
  if (now - lastDraw < interval) return;
  lastDraw = now;

  if (screen != lastScreen) {
    gfx->fillScreen(BLACK);
    lastScreen = screen;
  }

  if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
  switch (screen % LCD7B_NUM_SCREENS) {
    case 0: screenTargetIntel();   break;
    case 1: screenTop5();          break;
    case 2: screenRadar();         break;
    case 3: screenWeatherSystem(); break;
  }
  if (dataMutex) xSemaphoreGive(dataMutex);

  gfx->flush();
}

void checkTouch() {
  if (!touch.touched()) return;

  uint16_t x, y;
  touch.readData(&x, &y);

  if (millis() - lastTouchMs < 400) return;  // debounce, matches the other boards
  lastTouchMs = millis();

  int sx = constrain((int)x, 0, LCD_W - 1);
  int sy = constrain((int)y, 0, LCD_H - 1);

  if (screen % LCD7B_NUM_SCREENS == 2) {
    // Radar screen: check +/- range buttons in the left info column
    if (sx >= RNG_BTN_X && sx <= RNG_BTN_X + RNG_BTN_W &&
        sy >= RNG_PLUS_Y && sy <= RNG_PLUS_Y + RNG_BTN_H) {
      if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
      radarMaxKm = min(200.0f, radarMaxKm + 10.0f);
      if (configMutex) xSemaphoreGive(configMutex);
      markRangeDirty();
      Serial.printf("[lcd7b] Range increased: %d km\n", (int)radarMaxKm);
      return;
    }
    if (sx >= RNG_BTN_X && sx <= RNG_BTN_X + RNG_BTN_W &&
        sy >= RNG_MINUS_Y && sy <= RNG_MINUS_Y + RNG_BTN_H) {
      if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
      radarMaxKm = max(10.0f, radarMaxKm - 10.0f);
      if (configMutex) xSemaphoreGive(configMutex);
      markRangeDirty();
      Serial.printf("[lcd7b] Range decreased: %d km\n", (int)radarMaxKm);
      return;
    }
  }

  // Default: cycle to next screen
  screen = (screen + 1) % LCD7B_NUM_SCREENS;
}

// displayPanelSync is only meaningful on the RGB-parallel LCD-7B boards
// (driver restart after flash-write bursts); these panels need nothing.
void displayPanelSync() {}
