/*
 * JC4832W535 (ESP32-S3, 480x320, AXS15231B over QSPI) — full UI.
 *
 * Port of the CYD's screen set (Target Intel, Top 5, Radar, Weather &
 * System) onto Arduino_GFX. The CYD's two radar variants (compact "Radar
 * PPI" + borderless "Radar Full") exist only because the CYD's 320x240
 * canvas is too tight for one screen to do both well; this board's wider
 * 480x320 canvas doesn't have that constraint, so there's a single Radar
 * screen here with both a header and a bigger circle (R=145) than either
 * CYD variant achieves.
 *
 * Icon drawing and the label collision-avoidance helper are duplicated from
 * display_cyd.cpp rather than shared — Arduino_GFX (setCursor/print,
 * getTextBounds) and TFT_eSPI (sprites, drawString, textWidth/fontHeight)
 * differ enough that a shared abstraction would cost more than it saves.
 * There's no sprite here: Arduino_Canvas is a single full-panel framebuffer,
 * so every screen clears and redraws in full each frame (gfx->flush() at
 * the end) rather than relying on TFT_eSPI's opaque-text auto-erase.
 *
 * Pins, the AXS15231B_touch driver, and the native-320x480-plus-software-
 * rotation approach are all borrowed from the sibling JC3248W535 (same
 * QSPI/AXS15231B silicon family — see byte-me404/JC3248W535_display_test)
 * since no JC4832W535-specific reference was found. Touch offsets are still
 * the bring-up placeholders (full native-panel range) — real per-corner
 * calibration is a follow-up, not a blocker (see checkTouch()'s debug log).
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

// Screen set for this board: consolidated single Radar (see file header for
// why this differs from the CYD's 5-screen set with two radar variants).
#define JC_NUM_SCREENS 4  // Target Intel, Top 5, Radar, Weather & System

// Layout (480x320): header 0-25px; left info column x:0-140; radar area
// x:140-480 x y:25-320, circle centered in that region.
static const int HEADER_H = 25;
static const int RADAR_AREA_X = 140, RADAR_AREA_Y = HEADER_H;
static const int RADAR_AREA_W = 480 - RADAR_AREA_X, RADAR_AREA_H = 320 - HEADER_H;
static const int RADAR_CX = RADAR_AREA_X + RADAR_AREA_W / 2;
static const int RADAR_CY = RADAR_AREA_Y + RADAR_AREA_H / 2;
static const int RADAR_R = 145;

bool timeReady() { return time(nullptr) > 1700000000; }
void fmtClock(char* buf, size_t n, bool withSecs) {
  if (!timeReady()) { strncpy(buf, withSecs ? "--:--:--" : "--:--", n); return; }
  time_t t = time(nullptr); struct tm lt; localtime_r(&t, &lt);
  strftime(buf, n, withSecs ? "%H:%M:%S" : "%H:%M", &lt);
}

void drawHeader(const char* title) {
  char left[40];
  snprintf(left, sizeof(left), "%s [%d/%d]", title, screen + 1, JC_NUM_SCREENS);
  gfx->setTextSize(1);
  gfx->setTextColor(LIGHTGREY, BLACK);
  gfx->setCursor(10, 9);
  gfx->print(left);

  char t[10]; fmtClock(t, sizeof(t), true);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(t, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(470 - (int)tw, 9);
  gfx->print(t);

  gfx->drawLine(0, HEADER_H, 480, HEADER_H, DARKGREY);
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
// Ported from display_cyd.cpp's drawPlaneIcon — same geometry, drawn directly on
// the single global canvas (no per-screen sprite to parametrize over here).
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

// Rotor-cross glyph for rotorcraft (ADS-B category "A7"), ported from display_cyd.cpp.
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
// anchored in a ~20x18px box. Ported from display_cyd.cpp.
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
// label already placed this frame — same collision-avoidance algorithm as
// display_cyd.cpp's placeLabel(), adapted to getTextBounds() for sizing (no
// TFT_eSPI textWidth()/fontHeight() equivalent here) and to a caller-supplied
// bounding rect (no fixed-size sprite to clamp against — this is the radar
// area's rect within the shared full-panel canvas). All label lines use
// textSize(1); the caller must set that before calling.
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

  const int dx[6] = { 4, -4 - w, 4, -4 - w, 4, -4 - w };
  const int dy[6] = { -4, -4, 8, -8 - h, 12, 12 };

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
    gfx->setTextSize(3);
    gfx->setTextColor(RED, BLACK);
    gfx->setCursor(60, 130);
    gfx->print("NO TARGET IN RANGE");
    return;
  }

  gfx->setTextSize(3);
  gfx->setTextColor(GREEN, BLACK);
  gfx->setCursor(20, 40);
  gfx->print(nearest.callsign);

  gfx->setTextSize(1);
  gfx->setTextColor(CYAN, BLACK);
  gfx->setCursor(220, 48);
  if (nearest.hasRoute) gfx->printf("Route: %s -> %s", nearest.dep, nearest.arr);
  else gfx->print("Route: Unknown");

  gfx->setTextSize(2);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(20, 90);
  gfx->printf("Type: %s", nearest.country);

  gfx->setCursor(20, 120);
  gfx->printf("Distance: %.1f km (%s)", nearest.distanceKm, compass(nearest.bearingDeg));

  float altFeet = nearest.altitudeM * 3.28084f;
  gfx->setCursor(20, 150);
  if (nearest.onGround) gfx->print("Altitude: On Ground");
  else gfx->printf("Alt: %.0f ft (FL%03.0f)", altFeet, altFeet / 100.0f);

  gfx->setCursor(20, 180);
  gfx->printf("Speed: %.0f km/h", nearest.velocityMs * 3.6);

  int acx = 380, acy = 130, ar = 40;
  drawArrow(acx, acy, ar, nearest.trackDeg, CYAN);
  gfx->setTextSize(2);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(acx - 48, acy + ar + 15);
  gfx->printf("HDG %03.0f", nearest.trackDeg);
}

void screenTop5() {
  drawHeader("TOP 5 IN RANGE");

  if (top5Count == 0) {
    gfx->setTextSize(3);
    gfx->setTextColor(RED, BLACK);
    gfx->setCursor(150, 130);
    gfx->print("NO TARGETS");
    return;
  }

  gfx->setTextSize(1);
  gfx->setTextColor(CYAN, BLACK);
  gfx->setCursor(15, 35);
  gfx->print("FLIGHT");
  gfx->setCursor(150, 35);
  gfx->print("ROUTE");
  gfx->setCursor(280, 35);
  gfx->print("ALTITUDE");
  gfx->drawLine(10, 50, 470, 50, DARKGREY);

  int y = 65;
  gfx->setTextSize(2);
  for (int i = 0; i < top5Count; i++) {
    gfx->setTextColor(WHITE, BLACK);
    gfx->setCursor(15, y);
    gfx->print(top5[i].callsign);

    char rte[16];
    if (top5[i].hasRoute) snprintf(rte, sizeof(rte), "%s->%s", top5[i].dep, top5[i].arr);
    else strcpy(rte, "N/A");
    gfx->setCursor(150, y);
    gfx->print(rte);

    gfx->setCursor(280, y);
    if (top5[i].onGround) {
      gfx->print("Ground");
    } else {
      float ft = top5[i].altitudeM * 3.28084f;
      gfx->printf("%.0f ft (FL%03.0f)", ft, ft / 100.0f);
    }
    y += 40;
  }
}

// "-" / "+" button hit-rects, shared between the draw code below and checkTouch().
static const int RNG_BTN_X = 10, RNG_BTN_W = 55, RNG_BTN_H = 32;
static const int RNG_PLUS_Y = 210, RNG_MINUS_Y = 255;

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
  gfx->setTextSize(2);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(10, 35);
  gfx->printf("Contacts: %u", blipCount);

  gfx->setTextColor(CYAN, BLACK);
  gfx->setCursor(10, 60);
  if (nearest.valid) {
    gfx->print(nearest.callsign);
    gfx->setCursor(10, 85);
    gfx->printf("%.1f km", nearest.distanceKm);
  } else {
    gfx->print("No TGT");
  }

  gfx->setTextSize(1);
  if (weather.valid) {
    drawWeatherIcon(10, 115, weather.code);
    gfx->setTextColor(CYAN, BLACK);
    gfx->setCursor(35, 118);
    gfx->printf("%.1fC", weather.tempC);
    gfx->setCursor(10, 138);
    gfx->printf("Wind %.1f km/h", weather.windKmh);
  } else {
    gfx->setTextColor(CYAN, BLACK);
    gfx->setCursor(10, 138);
    gfx->print("Weather: --");
  }

  gfx->setTextSize(2);
  gfx->setTextColor(CYAN, BLACK);
  gfx->setCursor(10, 185);
  gfx->printf("RNG %d km", (int)rMax);

  gfx->fillRoundRect(RNG_BTN_X, RNG_PLUS_Y, RNG_BTN_W, RNG_BTN_H, 4, DARKGREY);
  gfx->setTextColor(WHITE, DARKGREY);
  gfx->setCursor(RNG_BTN_X + 18, RNG_PLUS_Y + 8);
  gfx->print("+");

  gfx->fillRoundRect(RNG_BTN_X, RNG_MINUS_Y, RNG_BTN_W, RNG_BTN_H, 4, DARKGREY);
  gfx->setCursor(RNG_BTN_X + 18, RNG_MINUS_Y + 8);
  gfx->print("-");

  // --- Radar circle ---
  const int cx = RADAR_CX, cy = RADAR_CY, R = RADAR_R;
  float elapsed = (millis() - lastDataMs) / 1000.0f;

  gfx->drawCircle(cx, cy, R, DARKGREY);
  gfx->drawCircle(cx, cy, R * 2 / 3, DARKGREY);
  gfx->drawCircle(cx, cy, R / 3, DARKGREY);
  gfx->drawLine(cx - R, cy, cx + R, cy, DARKGREY);
  gfx->drawLine(cx, cy - R, cx, cy + R, DARKGREY);

  float sweepDeg = fmodf(millis() / 15.0f, 360.0f);
  double sw = deg2rad(sweepDeg);
  gfx->drawLine(cx, cy, cx + (int)(sin(sw) * R), cy - (int)(cos(sw) * R), GREEN);

  gfx->setTextSize(1);

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

    // Every label field is independently toggled on/off, matching the CYD's
    // radar screens — nothing forced on, nothing shared between boards but
    // the toggle state itself (shared.h/data.cpp).
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
}

void screenWeatherSystem() {
  drawHeader("WEATHER & SYSTEM");
  gfx->drawLine(10, 150, 470, 150, DARKGREY);

  if (!weather.valid) {
    gfx->setTextSize(3);
    gfx->setTextColor(RED, BLACK);
    gfx->setCursor(30, 70);
    gfx->print("NO WX DATA");
  } else {
    gfx->setTextSize(3);
    gfx->setTextColor(YELLOW, BLACK);
    gfx->setCursor(30, 60);
    gfx->printf("%.1f C", weather.tempC);

    gfx->setTextSize(2);
    gfx->setTextColor(CYAN, BLACK);
    gfx->setCursor(260, 60);
    gfx->printf("Humidity: %d%%", weather.humidity);
    gfx->setCursor(260, 90);
    gfx->printf("Wind: %.0f km/h", weather.windKmh);
  }

  gfx->setTextSize(2);
  gfx->setTextColor(WHITE, BLACK);

  uint32_t up = millis() / 1000;
  gfx->setCursor(30, 175);
  gfx->printf("Uptime: %02lu:%02lu:%02lu", up / 3600, (up % 3600) / 60, up % 60);

  gfx->setCursor(260, 175);
  gfx->printf("WiFi RSSI: %d dBm", WiFi.RSSI());

  gfx->setCursor(30, 210);
  gfx->printf("RAM Free: %d KB", ESP.getFreeHeap() / 1024);

  gfx->setCursor(260, 210);
  gfx->printf("PSRAM Free: %d KB", ESP.getFreePsram() / 1024);

  gfx->setCursor(30, 245);
  gfx->printf("API Req: %lu OK / %lu FAIL", stats.requestsOk, stats.requestsFail);
}

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
  // invertDisplay is implemented on the panel driver, not the Arduino_Canvas
  // framebuffer wrapping it — Canvas doesn't override/forward it.
  gfxPanel->invertDisplay(invert);
}

#define BL_LEDC_CHANNEL 0
#define BL_LEDC_FREQ 5000
#define BL_LEDC_RES 8  // bits -> duty 0-255
void applyBrightness(uint8_t percent) {
  static bool ledcReady = false;
  if (!ledcReady) {
    ledcSetup(BL_LEDC_CHANNEL, BL_LEDC_FREQ, BL_LEDC_RES);
    ledcAttachPin(TFT_BL, BL_LEDC_CHANNEL);
    ledcReady = true;
  }
  ledcWrite(BL_LEDC_CHANNEL, (uint32_t)percent * 255 / 100);
}

void render() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 100) return;  // don't hammer the QSPI bus every loop() tick
  lastDraw = now;

  gfx->fillScreen(BLACK);

  if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
  switch (screen % JC_NUM_SCREENS) {
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

  // Debug: print touch coords for calibration (see file header note on
  // offsets still being bring-up placeholders).
  static uint32_t lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    Serial.printf("[jc4832] Touch: x=%d, y=%d\n", x, y);
    lastDebug = millis();
  }

  if (millis() - lastTouchMs < 400) return;  // debounce, matches the CYD's
  lastTouchMs = millis();

  int sx = constrain((int)x, 0, 479);
  int sy = constrain((int)y, 0, 319);

  if (screen % JC_NUM_SCREENS == 2) {
    // Radar screen: check +/- range buttons in the left info column
    if (sx >= RNG_BTN_X && sx <= RNG_BTN_X + RNG_BTN_W &&
        sy >= RNG_PLUS_Y && sy <= RNG_PLUS_Y + RNG_BTN_H) {
      if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
      radarMaxKm = min(200.0f, radarMaxKm + 10.0f);
      if (configMutex) xSemaphoreGive(configMutex);
      markRangeDirty();
      Serial.printf("[jc4832] Range increased: %d km\n", (int)radarMaxKm);
      return;
    }
    if (sx >= RNG_BTN_X && sx <= RNG_BTN_X + RNG_BTN_W &&
        sy >= RNG_MINUS_Y && sy <= RNG_MINUS_Y + RNG_BTN_H) {
      if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
      radarMaxKm = max(10.0f, radarMaxKm - 10.0f);
      if (configMutex) xSemaphoreGive(configMutex);
      markRangeDirty();
      Serial.printf("[jc4832] Range decreased: %d km\n", (int)radarMaxKm);
      return;
    }
  }

  // Default: cycle to next screen
  screen = (screen + 1) % JC_NUM_SCREENS;
}

// displayPanelSync is only meaningful on the RGB-parallel LCD-7B boards
// (driver restart after flash-write bursts); these panels need nothing.
void displayPanelSync() {}

uint8_t displayNumScreens() { return 4; }  // Target Intel, Top 5, Radar, Weather & System
