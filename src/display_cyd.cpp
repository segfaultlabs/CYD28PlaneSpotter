/*
 * CYD (ESP32-2432S028, TFT_eSPI/ILI9341, resistive XPT2046 touch) rendering
 * and touch handling. Implements the display interface declared in shared.h;
 * everything here is the only thing that changed by moving into its own
 * file — no behavior change from before the multi-board split.
 */

#include "shared.h"
#include <TFT_eSPI.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include "config.env"

// CYD Touch Pins
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

SPIClass touchSpi = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS);

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite radarSpr = TFT_eSprite(&tft);
// Separate sprite for the full-screen radar page, permanently allocated (8-bit
// color, ~53KB) rather than resized on demand from radarSpr — deleteSprite() +
// createSprite() on every screen transition fragmented the heap badly enough
// that later allocations silently failed and both radar screens stopped
// drawing at all. Two small permanent sprites are far safer than repeated
// alloc/free churn on this board's single, unified DRAM heap.
TFT_eSprite radarSprFull = TFT_eSprite(&tft);

bool timeReady() { return time(nullptr) > 1700000000; }
void fmtClock(char* buf, size_t n, bool withSecs) {
  if (!timeReady()) { strncpy(buf, withSecs ? "--:--:--" : "--:--", n); return; }
  time_t t = time(nullptr); struct tm lt; localtime_r(&t, &lt);
  strftime(buf, n, withSecs ? "%H:%M:%S" : "%H:%M", &lt);
}

void drawHeader(const char* title, bool newPage) {
  tft.setTextFont(2);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  
  if (newPage) {
    char left[30]; snprintf(left, sizeof(left), "%s [%d/%d]", title, screen + 1, NUM_SCREENS);
    tft.setTextPadding(200);
    tft.drawString(left, 10, 5);
    tft.drawLine(0, 25, 320, 25, TFT_DARKGREY);
  }
  
  char t[10]; fmtClock(t, sizeof(t), true);
  tft.setTextPadding(80);
  tft.drawString(t, 240, 5);
  tft.setTextPadding(0);
}

void drawArrow(int cx, int cy, int r, double angleDeg, uint16_t color) {
  double a = deg2rad(angleDeg);
  int tx = cx + (int)(sin(a) * r), ty = cy - (int)(cos(a) * r);
  int bx = cx - (int)(sin(a) * r), by = cy + (int)(cos(a) * r);
  tft.drawLine(bx, by, tx, ty, color);
  double left = a + deg2rad(150), right = a - deg2rad(150);
  tft.drawLine(tx, ty, tx + (int)(sin(left) * (r/2)), ty - (int)(cos(left) * (r/2)), color);
  tft.drawLine(tx, ty, tx + (int)(sin(right)* (r/2)), ty - (int)(cos(right)* (r/2)), color);
}

// Small heading-oriented plane glyph (fuselage + wings + tailfin) for radar blips
void drawPlaneIcon(TFT_eSprite &spr, int cx, int cy, double headingDeg, uint16_t color) {
  double h = deg2rad(headingDeg);
  double sh = sin(h), ch = cos(h);
  // Fuselage: nose to tail
  int noseX = cx + (int)(sh * 5),  noseY = cy - (int)(ch * 5);
  int tailX = cx - (int)(sh * 4),  tailY = cy + (int)(ch * 4);
  spr.drawLine(tailX, tailY, noseX, noseY, color);
  // Wings: perpendicular to heading, set slightly back from the nose
  int wingCx = cx + (int)(sh * 1), wingCy = cy - (int)(ch * 1);
  double pw = h + deg2rad(90);
  int wLx = wingCx + (int)(sin(pw) * 4), wLy = wingCy - (int)(cos(pw) * 4);
  int wRx = wingCx - (int)(sin(pw) * 4), wRy = wingCy + (int)(cos(pw) * 4);
  spr.drawLine(wLx, wLy, wRx, wRy, color);
  // Tailfin: small perpendicular stroke near the tail
  int fLx = tailX + (int)(sin(pw) * 2), fLy = tailY - (int)(cos(pw) * 2);
  int fRx = tailX - (int)(sin(pw) * 2), fRy = tailY + (int)(cos(pw) * 2);
  spr.drawLine(fLx, fLy, fRx, fRy, color);
}

// Rotor-cross glyph for rotorcraft (ADS-B category "A7") — visually distinct from
// the fixed-wing plane icon. Rotor disc is drawn fixed (rotors spin, so a heading-
// oriented rotor would be misleading); only the short tail boom follows heading.
void drawHelicopterIcon(TFT_eSprite &spr, int cx, int cy, double headingDeg, uint16_t color) {
  double h = deg2rad(headingDeg);
  int tailX = cx - (int)(sin(h) * 5), tailY = cy + (int)(cos(h) * 5);
  spr.drawLine(cx, cy, tailX, tailY, color);
  spr.drawLine(cx - 4, cy, cx + 4, cy, color);
  spr.drawLine(cx, cy - 4, cx, cy + 4, color);
  spr.fillCircle(cx, cy, 2, color);
}

// Shared by both radar screens: places a multi-line blip label near (bx,by),
// avoiding overlap with every label already placed this frame (tracked in
// `placed`/`placedCount`, reset once per frame by the caller). Lines can mix
// fonts (e.g. a larger callsign line) — width/height come from the sprite's
// actual font metrics, not a hardcoded guess. Tries a handful of fallback
// offsets (right, left, below, above) before giving up; if none clear, the
// label is simply skipped for this frame rather than drawn overlapping —
// aircraft move and the sweep highlight changes constantly, so a label that's
// briefly hidden isn't a real loss.
struct LabelRect { int x, y, w, h; };

bool placeLabel(TFT_eSprite &spr, LabelRect *placed, int &placedCount, int maxPlaced,
                 int bx, int by, char lines[][12], const uint8_t *lineFonts, int lineCount,
                 int spriteSize) {
  if (lineCount == 0) return false;

  int w = 0, h = 0;
  int lineY[9];
  for (int i = 0; i < lineCount; i++) {
    spr.setTextFont(lineFonts[i]);
    int lw = spr.textWidth(lines[i]);
    if (lw > w) w = lw;
    lineY[i] = h;
    h += spr.fontHeight(lineFonts[i]);
  }
  w += 3;

  const int dx[6] = { 4, -4 - w, 4, -4 - w, 4, -4 - w };
  const int dy[6] = { -4, -4, 8, -8 - h, 12, 12 };

  for (int attempt = 0; attempt < 6; attempt++) {
    int lx = bx + dx[attempt];
    int ly = by + dy[attempt];
    if (lx < 0) lx = 0;
    if (lx + w > spriteSize) lx = spriteSize - w;
    if (ly < 0) ly = 0;
    if (ly + h > spriteSize) ly = spriteSize - h;
    if (lx < 0 || ly < 0) continue;  // label too big for the sprite at all

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
      for (int li = 0; li < lineCount; li++) {
        spr.setTextFont(lineFonts[li]);
        spr.drawString(lines[li], lx, ly + lineY[li]);
      }
      if (placedCount < maxPlaced) placed[placedCount++] = cand;
      return true;
    }
  }
  return false;
}

// Small cloud glyph, top-left anchored, ~20px wide x 10px tall
void drawCloudShape(int x, int y, uint16_t color) {
  tft.fillCircle(x + 5, y + 6, 4, color);
  tft.fillCircle(x + 10, y + 4, 5, color);
  tft.fillCircle(x + 15, y + 6, 4, color);
  tft.fillRect(x + 3, y + 6, 14, 5, color);
}

// Maps an Open-Meteo WMO weather_code to a compact vector icon, top-left
// anchored in a ~20x18px box.
void drawWeatherIcon(int x, int y, int code) {
  if (code == 0) {
    // Clear sky: sun with rays
    int cx = x + 9, cy = y + 8;
    tft.fillCircle(cx, cy, 5, TFT_YELLOW);
    for (int a = 0; a < 360; a += 45) {
      double r = deg2rad(a);
      tft.drawLine(cx + (int)(sin(r) * 7), cy - (int)(cos(r) * 7),
                   cx + (int)(sin(r) * 10), cy - (int)(cos(r) * 10), TFT_YELLOW);
    }
  } else if (code >= 1 && code <= 3) {
    // Mainly clear / partly cloudy / overcast: sun peeking behind a cloud
    int cx = x + 5, cy = y + 4;
    tft.fillCircle(cx, cy, 4, TFT_YELLOW);
    drawCloudShape(x + 2, y + 5, TFT_LIGHTGREY);
  } else if (code == 45 || code == 48) {
    // Fog: stacked horizontal bands
    for (int i = 0; i < 3; i++)
      tft.drawFastHLine(x, y + 5 + i * 4, 18, TFT_LIGHTGREY);
  } else if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82)) {
    // Drizzle / rain / rain showers
    drawCloudShape(x, y, TFT_LIGHTGREY);
    for (int i = 0; i < 3; i++)
      tft.drawLine(x + 4 + i * 5, y + 12, x + 2 + i * 5, y + 17, TFT_CYAN);
  } else if ((code >= 71 && code <= 77) || code == 85 || code == 86) {
    // Snow fall / snow showers
    drawCloudShape(x, y, TFT_LIGHTGREY);
    for (int i = 0; i < 3; i++) {
      int sx = x + 4 + i * 5, sy = y + 15;
      tft.drawLine(sx - 2, sy, sx + 2, sy, TFT_WHITE);
      tft.drawLine(sx, sy - 2, sx, sy + 2, TFT_WHITE);
    }
  } else if (code >= 95) {
    // Thunderstorm
    drawCloudShape(x, y, TFT_DARKGREY);
    tft.drawLine(x + 10, y + 11, x + 7, y + 16, TFT_YELLOW);
    tft.drawLine(x + 7, y + 16, x + 11, y + 16, TFT_YELLOW);
    tft.drawLine(x + 11, y + 16, x + 8, y + 21, TFT_YELLOW);
  } else {
    // Unknown/overcast fallback
    drawCloudShape(x, y, TFT_LIGHTGREY);
  }
}


void screenTargetIntel(bool newPage) {
  drawHeader("TARGET INTEL", newPage);
  tft.setTextPadding(300);
  
  if (!nearest.valid) {
    tft.setTextFont(4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("NO TARGET IN RANGE", 40, 100);
    return;
  }

  tft.setTextFont(4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString(nearest.callsign, 20, 40);
  
  tft.setTextFont(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  if (nearest.hasRoute) {
    char rte[32]; snprintf(rte, sizeof(rte), "Route: %s -> %s", nearest.dep, nearest.arr);
    tft.drawString(rte, 160, 45);
  } else {
    tft.drawString("Route: Unknown", 160, 45);
  }
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  char buf[64];
  snprintf(buf, sizeof(buf), "Type: %s", nearest.country);
  tft.drawString(buf, 20, 80);
  
  snprintf(buf, sizeof(buf), "Distance: %.1f km (%s)", nearest.distanceKm, compass(nearest.bearingDeg));
  tft.drawString(buf, 20, 110);
  
  float altFeet = nearest.altitudeM * 3.28084f;
  if (nearest.onGround) snprintf(buf, sizeof(buf), "Altitude: On Ground");
  else snprintf(buf, sizeof(buf), "Alt: %.0f ft (FL%03.0f)", altFeet, altFeet / 100.0f);
  tft.drawString(buf, 20, 140);
  
  snprintf(buf, sizeof(buf), "Speed: %.0f km/h", nearest.velocityMs * 3.6);
  tft.drawString(buf, 20, 170);

  tft.fillRect(220, 110, 60, 60, TFT_BLACK); 
  drawArrow(250, 140, 25, nearest.trackDeg, TFT_CYAN);
  snprintf(buf, sizeof(buf), "HDG %03.0f", nearest.trackDeg);
  tft.drawString(buf, 225, 180);
  
  tft.setTextPadding(0);
}

void screenTop5(bool newPage) {
  drawHeader("TOP 5 IN RANGE", newPage);
  
  if (top5Count == 0) {
    tft.setTextFont(4); tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("NO TARGETS", 100, 100);
    return;
  }
  
  if (newPage) {
    tft.setTextFont(2); tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("FLIGHT", 10, 40);
    tft.drawString("ROUTE", 100, 40);
    tft.drawString("ALTITUDE", 190, 40);
    tft.drawLine(10, 60, 310, 60, TFT_DARKGREY);
  }
  
  tft.setTextFont(2); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(80); 
  
  int y = 70;
  for (int i=0; i<5; i++) {
    if (i < top5Count) {
      tft.drawString(top5[i].callsign, 10, y);
      
      char rte[16];
      if (top5[i].hasRoute) snprintf(rte, sizeof(rte), "%s->%s", top5[i].dep, top5[i].arr);
      else strcpy(rte, "N/A");
      tft.drawString(rte, 100, y);
      
      char alt[32];
      tft.setTextPadding(120); 
      if (top5[i].onGround) strcpy(alt, "Ground");
      else {
        float ft = top5[i].altitudeM * 3.28084f;
        snprintf(alt, sizeof(alt), "%.0f ft (FL%03.0f)", ft, ft/100.0f);
      }
      tft.drawString(alt, 190, y);
      tft.setTextPadding(80);
      
    } else {
      tft.drawString(" ", 10, y);
      tft.drawString(" ", 100, y);
      tft.setTextPadding(120);
      tft.drawString(" ", 190, y);
      tft.setTextPadding(80);
    }
    y += 30;
  }
  tft.setTextPadding(0);
}

void screenRadar(bool newPage) {
  drawHeader("RADAR PPI", newPage);

  // Snapshot config under mutex
  double hLat, hLon; float rMax;
  bool showCS, showAir, showSpd, showFlt, showRte, showRg, showSq, showVr, showTy;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; rMax = radarMaxKm;
  showCS = showCallsign; showAir = showAirline; showSpd = showSpeed; showFlt = showFL; showRte = showRoute;
  showRg = showReg; showSq = showSquawk; showVr = showVRate; showTy = showType;
  if (configMutex) xSemaphoreGive(configMutex);

  tft.setTextFont(2); tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextPadding(110);
  char buf[32]; snprintf(buf, sizeof(buf), "Contacts: %u", blipCount);
  tft.drawString(buf, 5, 50);
  if(nearest.valid) {
    snprintf(buf, sizeof(buf), "TGT: %s", nearest.callsign);
    tft.drawString(buf, 5, 80);
    snprintf(buf, sizeof(buf), "%.1f km", nearest.distanceKm);
    tft.drawString(buf, 5, 110);
  } else {
    tft.drawString("No TGT", 5, 80);
    tft.drawString(" ", 5, 110);
  }
  tft.setTextPadding(0);

  // Weather readout (icon + text) in the otherwise-empty space above the range indicator.
  // Icon area is redrawn from a black-filled rect each frame — icon shapes vary in size/
  // position by weather code, so (unlike text) there's no opaque-background auto-erase.
  tft.fillRect(5, 128, 26, 26, TFT_BLACK);
  if (weather.valid) drawWeatherIcon(8, 130, weather.code);

  tft.setTextFont(1); tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.setTextPadding(110);
  if (weather.valid) {
    snprintf(buf, sizeof(buf), "%.1fC  %d%%RH", weather.tempC, weather.humidity);
    tft.drawString(buf, 5, 158);
    snprintf(buf, sizeof(buf), "Wind %.1f km/h", weather.windKmh);
    tft.drawString(buf, 5, 173);
  } else {
    tft.drawString("Weather: --", 5, 158);
    tft.drawString(" ", 5, 173);
  }
  tft.setTextPadding(0);

  const int cx = 100, cy = 100, R = 95;
  float elapsed = (millis() - lastDataMs) / 1000.0f;

  radarSpr.fillSprite(TFT_BLACK);
  radarSpr.drawCircle(cx, cy, R, TFT_DARKGREY);
  radarSpr.drawCircle(cx, cy, R*2/3, TFT_DARKGREY);
  radarSpr.drawCircle(cx, cy, R/3, TFT_DARKGREY);
  radarSpr.drawLine(cx-R, cy, cx+R, cy, TFT_DARKGREY);
  radarSpr.drawLine(cx, cy-R, cx, cy+R, TFT_DARKGREY);

  float sweepDeg = fmodf(millis() / 15.0f, 360.0f);
  double sw = deg2rad(sweepDeg);
  radarSpr.drawLine(cx, cy, cx + (int)(sin(sw)*R), cy - (int)(cos(sw)*R), TFT_GREEN);

  radarSpr.setTextFont(1);
  radarSpr.setTextColor(TFT_WHITE, TFT_BLACK);

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
    uint16_t blipColor = (behind < 30) ? TFT_YELLOW : TFT_GREEN;
    if (strcmp(blips[i].category, "A7") == 0)
      drawHelicopterIcon(radarSpr, bx, by, blips[i].track, blipColor);
    else
      drawPlaneIcon(radarSpr, bx, by, blips[i].track, blipColor);

    // Label: every field, including the flight number itself, is independently
    // toggled on/off — nothing is forced on. Lines are collected first so we know
    // the total stack height and can fit the whole thing inside the sprite bounds
    // (clamping each line independently, as before, would make several lines
    // overlap at the same bottom edge once more fields are enabled). If nothing
    // is toggled on (or none of it has data for this aircraft), no label is drawn.
    {
      char lines[9][12];
      uint8_t lineFonts[9];
      int lineCount = 0;
      bool hasCallsign = blips[i].callsign[0] && strcmp(blips[i].callsign, "(no id)") != 0;

      if (showCS && hasCallsign) {
        strncpy(lines[lineCount], blips[i].callsign, 11); lines[lineCount][11] = '\0';
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showFlt) {
        float altFt = blips[i].altitudeM * 3.28084f;
        snprintf(lines[lineCount], 12, "FL%03.0f", altFt / 100.0f);
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showRte && blips[i].hasRoute) {
        snprintf(lines[lineCount], 12, "%s>%s", blips[i].dep, blips[i].arr);
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showAir) {
        const char* airline = blips[i].airline[0] ? blips[i].airline : lookupAirline(blips[i].callsign);
        if (airline) {
          strncpy(lines[lineCount], airline, 10); lines[lineCount][10] = '\0';
          lineFonts[lineCount] = 1; lineCount++;
        }
      }
      if (showRg && blips[i].reg[0]) {
        strncpy(lines[lineCount], blips[i].reg, 11); lines[lineCount][11] = '\0';
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showSq && blips[i].squawk[0]) {
        snprintf(lines[lineCount], 12, "SQ%s", blips[i].squawk);
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showVr) {
        snprintf(lines[lineCount], 12, "%+dfpm", blips[i].vrateFpm);
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showTy && blips[i].typeCode[0]) {
        strncpy(lines[lineCount], blips[i].typeCode, 11); lines[lineCount][11] = '\0';
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showSpd) {
        float speedKt = blips[i].speedMs * 1.94384f;
        snprintf(lines[lineCount], 12, "%ukn", (unsigned int)(speedKt + 0.5f));
        lineFonts[lineCount] = 1; lineCount++;
      }

      placeLabel(radarSpr, placed, placedCount, MAX_BLIPS, bx, by, lines, lineFonts, lineCount, 200);
    }
  }
  radarSpr.pushSprite(115, 35);

  // --- Range indicator & +/- buttons (bottom-left, side by side) ---
  // Range label
  tft.setTextFont(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(80);
  char rng[16]; snprintf(rng, sizeof(rng), "RNG %d km", (int)rMax);
  tft.drawString(rng, 10, 193);

  // "-" button (left, 30x20)
  tft.fillRoundRect(10, 215, 30, 20, 4, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextFont(2);
  tft.setTextPadding(30);
  tft.drawString("-", 20, 216);

  // "+" button (right of "-", 30x20)
  tft.fillRoundRect(50, 215, 30, 20, 4, TFT_DARKGREY);
  tft.drawString("+", 60, 216);

  tft.setTextPadding(0);
}

// Full-screen radar: no header/clock, circle sized to the screen's true constraint
// (height, since it's a 320x240 landscape panel) so it's as large as it can be while
// staying circular. Weather (icon + temp only) lives in the narrow left margin, range
// + its +/- buttons in the narrow right margin, both left over once the circle claims
// the full height.
void screenRadarFull(bool newPage) {
  const int R = 117, SPR = 234;
  const int sprX = (320 - SPR) / 2, sprY = (240 - SPR) / 2;  // = 43, 3
  const int cx = R, cy = R;

  if (newPage) {
    tft.fillScreen(TFT_BLACK);
  }

  double hLat, hLon; float rMax;
  bool showCS, showAir, showSpd, showFlt, showRte, showRg, showSq, showVr, showTy;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; rMax = radarMaxKm;
  showCS = showCallsign; showAir = showAirline; showSpd = showSpeed; showFlt = showFL; showRte = showRoute;
  showRg = showReg; showSq = showSquawk; showVr = showVRate; showTy = showType;
  if (configMutex) xSemaphoreGive(configMutex);

  float elapsed = (millis() - lastDataMs) / 1000.0f;

  radarSprFull.fillSprite(TFT_BLACK);
  radarSprFull.drawCircle(cx, cy, R, TFT_DARKGREY);
  radarSprFull.drawCircle(cx, cy, R*2/3, TFT_DARKGREY);
  radarSprFull.drawCircle(cx, cy, R/3, TFT_DARKGREY);
  radarSprFull.drawLine(cx-R, cy, cx+R, cy, TFT_DARKGREY);
  radarSprFull.drawLine(cx, cy-R, cx, cy+R, TFT_DARKGREY);

  float sweepDeg = fmodf(millis() / 15.0f, 360.0f);
  double sw = deg2rad(sweepDeg);
  radarSprFull.drawLine(cx, cy, cx + (int)(sin(sw)*R), cy - (int)(cos(sw)*R), TFT_GREEN);

  radarSprFull.setTextColor(TFT_WHITE, TFT_BLACK);

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
    uint16_t blipColor = (behind < 30) ? TFT_YELLOW : TFT_GREEN;
    if (strcmp(blips[i].category, "A7") == 0)
      drawHelicopterIcon(radarSprFull, bx, by, blips[i].track, blipColor);
    else
      drawPlaneIcon(radarSprFull, bx, by, blips[i].track, blipColor);

    {
      char lines[9][12];
      uint8_t lineFonts[9];
      int lineCount = 0;
      bool hasCallsign = blips[i].callsign[0] && strcmp(blips[i].callsign, "(no id)") != 0;

      if (showCS && hasCallsign) {
        strncpy(lines[lineCount], blips[i].callsign, 11); lines[lineCount][11] = '\0';
        lineFonts[lineCount] = 2; lineCount++;  // bigger font for the callsign line only
      }
      if (showFlt) {
        float altFt = blips[i].altitudeM * 3.28084f;
        snprintf(lines[lineCount], 12, "FL%03.0f", altFt / 100.0f);
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showRte && blips[i].hasRoute) {
        snprintf(lines[lineCount], 12, "%s>%s", blips[i].dep, blips[i].arr);
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showAir) {
        const char* airline = blips[i].airline[0] ? blips[i].airline : lookupAirline(blips[i].callsign);
        if (airline) {
          strncpy(lines[lineCount], airline, 10); lines[lineCount][10] = '\0';
          lineFonts[lineCount] = 1; lineCount++;
        }
      }
      if (showRg && blips[i].reg[0]) {
        strncpy(lines[lineCount], blips[i].reg, 11); lines[lineCount][11] = '\0';
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showSq && blips[i].squawk[0]) {
        snprintf(lines[lineCount], 12, "SQ%s", blips[i].squawk);
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showVr) {
        snprintf(lines[lineCount], 12, "%+dfpm", blips[i].vrateFpm);
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showTy && blips[i].typeCode[0]) {
        strncpy(lines[lineCount], blips[i].typeCode, 11); lines[lineCount][11] = '\0';
        lineFonts[lineCount] = 1; lineCount++;
      }
      if (showSpd) {
        float speedKt = blips[i].speedMs * 1.94384f;
        snprintf(lines[lineCount], 12, "%ukn", (unsigned int)(speedKt + 0.5f));
        lineFonts[lineCount] = 1; lineCount++;
      }

      placeLabel(radarSprFull, placed, placedCount, MAX_BLIPS, bx, by, lines, lineFonts, lineCount, SPR);
    }
  }
  radarSprFull.pushSprite(sprX, sprY);

  // Left margin, bottom: weather icon + temperature + wind (humidity stays on the
  // dedicated Weather & System screen — this strip is only ~43px wide).
  tft.fillRect(0, 168, sprX, 72, TFT_BLACK);
  if (weather.valid) {
    drawWeatherIcon(10, 170, weather.code);
    tft.setTextFont(1); tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.setTextPadding(sprX);
    char tbuf[8]; snprintf(tbuf, sizeof(tbuf), "%.0fC", weather.tempC);
    tft.drawString(tbuf, 2, 194);
    char wbuf[10]; snprintf(wbuf, sizeof(wbuf), "%.0fkm/h", weather.windKmh);
    tft.drawString(wbuf, 2, 206);
    tft.setTextPadding(0);
  }

  // Right margin: range value + "+" button at the top, "-" button at the bottom.
  int rx = sprX + SPR;  // = 277
  tft.setTextFont(1); tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.setTextPadding(320 - rx);
  char rng[12]; snprintf(rng, sizeof(rng), "%dkm", (int)rMax);
  tft.drawString(rng, rx + 2, 5);
  tft.setTextPadding(0);

  tft.fillRoundRect(rx + 2, 25, 36, 30, 4, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextFont(2);
  tft.setTextPadding(36);
  tft.drawString("+", rx + 20, 31);

  tft.fillRoundRect(rx + 2, 205, 36, 30, 4, TFT_DARKGREY);
  tft.drawString("-", rx + 20, 211);
  tft.setTextPadding(0);
}

// Unified Weather and System Screen
void screenWeatherSystem(bool newPage) {
  drawHeader("WEATHER & SYSTEM", newPage);
  
  if (newPage) {
    // Divider line between weather and system info
    tft.drawLine(10, 115, 310, 115, TFT_DARKGREY);
  }

  // --- TOP HALF: WEATHER ---
  if (!weather.valid) { 
    tft.setTextFont(4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("NO WX DATA", 20, 50); 
  } else {
    tft.setTextFont(4);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextPadding(100);
    char buf[32]; snprintf(buf, sizeof(buf), "%.1f C", weather.tempC);
    tft.drawString(buf, 20, 45);

    tft.setTextFont(2); 
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextPadding(150);
    snprintf(buf, sizeof(buf), "Humidity: %d%%", weather.humidity);
    tft.drawString(buf, 160, 45);
    snprintf(buf, sizeof(buf), "Wind: %.0f km/h", weather.windKmh);
    tft.drawString(buf, 160, 75);
  }

  // --- BOTTOM HALF: SYSTEM ---
  tft.setTextFont(2); 
  tft.setTextColor(TFT_WHITE, TFT_BLACK); 
  tft.setTextPadding(160);
  
  char bufSys[64];
  uint32_t up = millis() / 1000;
  snprintf(bufSys, sizeof(bufSys), "Uptime: %02lu:%02lu:%02lu", up / 3600, (up % 3600) / 60, up % 60);
  tft.drawString(bufSys, 20, 130);
  
  snprintf(bufSys, sizeof(bufSys), "WiFi RSSI: %d dBm", WiFi.RSSI());
  tft.drawString(bufSys, 180, 130);
  
  snprintf(bufSys, sizeof(bufSys), "RAM Free: %d KB", ESP.getFreeHeap() / 1024);
  tft.drawString(bufSys, 20, 160);
  
  tft.setTextPadding(300); // Larger padding for API count
  snprintf(bufSys, sizeof(bufSys), "API Req: %lu OK / %lu FAIL", stats.requestsOk, stats.requestsFail);
  tft.drawString(bufSys, 20, 190);
  
  tft.setTextPadding(0);
}


void render() {
  bool newPage = false;
  if (screen != lastScreen) {
    tft.fillScreen(TFT_BLACK);
    lastScreen = screen;
    newPage = true;
  }
  
  // Take mutex for the duration of rendering so we see a consistent snapshot
  if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
  switch (screen) {
    case 0: screenTargetIntel(newPage); break;
    case 1: screenTop5(newPage);        break;
    case 2: screenRadar(newPage);       break;
    case 3: screenRadarFull(newPage);   break;
    case 4: screenWeatherSystem(newPage); break;
  }
  if (dataMutex) xSemaphoreGive(dataMutex);
}


void checkTouch() {
  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    
    // STRICT FILTER: 
    // 1. p.z > 400 ignores light ghost touches and case pinching.
    // 2. p.x and p.y bounds ignore the 0 and 4095 coordinate spikes caused by SPI noise.
    if (p.z > 400 && p.x > 100 && p.x < 4000 && p.y > 100 && p.y < 4000) {
      
      if (millis() - lastTouchMs > 400) { // 400ms debounce to prevent double-skips
        lastTouchMs = millis();

        // Map raw touch to screen coordinates (CYD rotation 1)
        // Calibrated: raw ~200-3900, X-axis inverted for this panel
        int sx = 320 - (int)((p.y - 200.0f) * 320.0f / 3700.0f);
        int sy = 240 - (int)((p.x - 200.0f) * 240.0f / 3700.0f);
        sx = constrain(sx, 0, 319);
        sy = constrain(sy, 0, 239);

        // Debug: print touch coords for calibration
        static uint32_t lastDebug = 0;
        if (millis() - lastDebug > 2000) {
          Serial.printf("Touch: raw(%d,%d,%d) -> screen(%d,%d)\n", p.x, p.y, p.z, sx, sy);
          lastDebug = millis();
        }

        // Screen-specific touch handling
        if (screen == 2) {
          // Radar screen: check +/- range buttons (bottom-left, stacked vertically)
          // "+" button: sx=10..50, sy=160..205
          if (sx >= 10 && sx <= 50 && sy >= 160 && sy <= 205) {
            if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
            radarMaxKm = min(200.0f, radarMaxKm + 10.0f);
            if (configMutex) xSemaphoreGive(configMutex);
            markRangeDirty();
            Serial.printf("Range increased: %d km\n", (int)radarMaxKm);
            lastScreenSwap = millis();
            return;
          }
          // "-" button: sx=10..50, sy=206..240
          if (sx >= 10 && sx <= 50 && sy >= 206 && sy <= 240) {
            if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
            radarMaxKm = max(10.0f, radarMaxKm - 10.0f);
            if (configMutex) xSemaphoreGive(configMutex);
            markRangeDirty();
            Serial.printf("Range decreased: %d km\n", (int)radarMaxKm);
            lastScreenSwap = millis();
            return;
          }
        } else if (screen == 3) {
          // Radar Full screen: "+" button top-right, "-" button bottom-right
          // "+" button: sx=279..315, sy=25..55
          if (sx >= 279 && sx <= 315 && sy >= 25 && sy <= 55) {
            if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
            radarMaxKm = min(200.0f, radarMaxKm + 10.0f);
            if (configMutex) xSemaphoreGive(configMutex);
            markRangeDirty();
            Serial.printf("Range increased: %d km\n", (int)radarMaxKm);
            lastScreenSwap = millis();
            return;
          }
          // "-" button: sx=279..315, sy=205..235
          if (sx >= 279 && sx <= 315 && sy >= 205 && sy <= 235) {
            if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
            radarMaxKm = max(10.0f, radarMaxKm - 10.0f);
            if (configMutex) xSemaphoreGive(configMutex);
            markRangeDirty();
            Serial.printf("Range decreased: %d km\n", (int)radarMaxKm);
            lastScreenSwap = millis();
            return;
          }
        }

        // Default: cycle to next screen
        screen = (screen + 1) % NUM_SCREENS;
        lastScreenSwap = millis();
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Display interface implementation (declared in shared.h)
// ---------------------------------------------------------------------------
void displaySetup() {
  touchSpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, -1);
  ts.begin(touchSpi);
  ts.setRotation(1);

  tft.begin();
  tft.setRotation(1);

  // Both sprites at 8-bit color (not 16-bit) to keep combined permanent memory
  // small enough to leave room for WiFi/TLS buffers — see radarSprFull's
  // declaration comment for why two permanent sprites beat resizing one.
  radarSpr.setColorDepth(8);
  radarSpr.createSprite(200, 200);
  radarSprFull.setColorDepth(8);
  radarSprFull.createSprite(234, 234);

  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(4); tft.setTextColor(TFT_GREEN);
  tft.drawString("CYD PLANE SPOTTER", 20, 100);
  delay(1500);
}

void connectWiFiShow() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(4);
  tft.drawString("Connecting WiFi...", 20, 100);
  connectWiFi();
  tft.fillScreen(TFT_BLACK);
}

void applyInvertColors(bool invert) {
  tft.invertDisplay(invert);
}

// TFT_eSPI drives TFT_BL as a plain digitalWrite HIGH via its own init (see
// TFT_BACKLIGHT_ON build flag) -- ledcAttachPin here takes the pin away from
// that and puts it under PWM instead, so brightness can be graduated.
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
