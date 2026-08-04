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
#include <WiFi.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include "IOExtension.h"
#include "GT911_touch.h"
#include "qrcodegen.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"

// Arduino_GFX 1.6.7 removed the short color aliases (BLACK, WHITE, ...) that
// existed in 1.4.5 (used by display_jc4832.cpp / display_lcd7b.cpp), keeping
// only the RGB565_-prefixed names. Aliasing locally rather than rewriting
// every color reference throughout this file.
#define BLACK RGB565_BLACK
#define WHITE RGB565_WHITE
#define RED RGB565_RED
#define GREEN RGB565_GREEN
#define CYAN RGB565_CYAN
#define YELLOW RGB565_YELLOW
#define DARKGREY RGB565_DARKGREY
#define LIGHTGREY RGB565_LIGHTGREY
#define MAGENTA RGB565_MAGENTA
#define ORANGE 0xFD20

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

// Layout (1024x600): header 0-35px; left info column x:0-260; radar area
// x:260-1024 x y:35-600, circle centered in that region. Declared up here
// (not down with the rest of the layout section) since the sweep/present
// machinery below is declared before the layout section proper.
static const int HEADER_H = 35;
static const int RADAR_AREA_X = 260, RADAR_AREA_Y = HEADER_H;
static const int RADAR_AREA_W = LCD_W - RADAR_AREA_X, RADAR_AREA_H = LCD_H - HEADER_H;
static const int RADAR_CX = RADAR_AREA_X + RADAR_AREA_W / 2;
static const int RADAR_CY = RADAR_AREA_Y + RADAR_AREA_H / 2;
static const int RADAR_R = 260;

// Two radar layouts share one implementation: the classic view (left info
// column + circle in the remaining area) and the full-screen view (no
// header/column, circle fills the canvas, big touch buttons in the margins).
// Which one is active depends on the current screen index — see
// radarLayout().
struct RadarLayout {
  int cx, cy, R;                       // circle geometry
  int boundX, boundY, boundW, boundH;  // label-placement bounds
  bool full;                           // full-screen variant
};
static RadarLayout radarLayout(uint8_t scr) {
  if (scr == 4) return { 512, 300, 285, 0, 0, LCD_W, LCD_H, true };
  return { RADAR_CX, RADAR_CY, RADAR_R, RADAR_AREA_X, RADAR_AREA_Y, RADAR_AREA_W, RADAR_AREA_H, false };
}

// Physical airports plotted on the radar. Static table (IATA + coordinates)
// — there is no practical live source for "airports near an arbitrary
// point", and a static table is instant, works offline, and costs ~1.5KB of
// flash. Weighted toward East Asia (primary dev location) plus world hubs.
// Coordinates are airport reference points; at radar scale (10-200km) a few
// hundred meters of error is invisible.
struct AirportPt { const char *iata; float lat; float lon; };
static const AirportPt AIRPORTS[] = {
  // Korea / East Asia
  { "ICN", 37.4602f, 126.4407f }, { "GMP", 37.5583f, 126.7906f },
  { "CJJ", 36.7166f, 127.4991f }, { "PUS", 35.1795f, 128.9382f },
  { "CJU", 33.5113f, 126.4930f }, { "TAE", 35.8941f, 128.6589f },
  { "KWJ", 35.1264f, 126.8089f }, { "YNY", 38.0613f, 128.6691f },
  { "OSN", 37.0906f, 127.0296f },
  { "NRT", 35.7720f, 140.3929f }, { "HND", 35.5494f, 139.7798f },
  { "KIX", 34.4342f, 135.2441f }, { "FUK", 33.5859f, 130.4500f },
  { "CTS", 42.7752f, 141.6923f }, { "NGO", 34.8584f, 136.8053f },
  { "OKA", 26.1958f, 127.6458f },
  { "PEK", 40.0799f, 116.6031f }, { "PKX", 39.5098f, 116.4105f },
  { "PVG", 31.1443f, 121.8083f }, { "SHA", 31.1979f, 121.3363f },
  { "HKG", 22.3080f, 113.9185f }, { "MFM", 22.1496f, 113.5916f },
  { "TPE", 25.0797f, 121.2342f }, { "TSA", 25.0697f, 121.5525f },
  { "CAN", 23.3924f, 113.2988f }, { "SZX", 22.6393f, 113.8106f },
  { "CTU", 30.5785f, 103.9471f }, { "XIY", 34.4471f, 108.7516f },
  // North America
  { "LAX", 33.9416f, -118.4085f }, { "JFK", 40.6413f, -73.7781f },
  { "SFO", 37.6213f, -122.3790f }, { "ORD", 41.9742f, -87.9073f },
  { "ATL", 33.6407f, -84.4277f }, { "DFW", 32.8998f, -97.0403f },
  { "DEN", 39.8561f, -104.6737f }, { "SEA", 47.4502f, -122.3088f },
  { "MIA", 25.7959f, -80.2870f }, { "BOS", 42.3656f, -71.0096f },
  { "LAS", 36.0840f, -115.1537f }, { "YVR", 49.1967f, -123.1815f },
  { "YYZ", 43.6777f, -79.6248f }, { "MEX", 19.4363f, -99.0721f },
  { "HNL", 21.3245f, -157.9251f }, { "ANC", 61.1743f, -149.9962f },
  // South America
  { "GRU", -23.4356f, -46.4731f }, { "EZE", -34.8222f, -58.5358f },
  { "SCL", -33.3930f, -70.7858f },
  // Europe
  { "LHR", 51.4700f, -0.4543f }, { "LGW", 51.1537f, -0.1821f },
  { "CDG", 49.0097f, 2.5479f }, { "FRA", 50.0379f, 8.5622f },
  { "AMS", 52.3105f, 4.7683f }, { "MAD", 40.4983f, -3.5676f },
  { "FCO", 41.8003f, 12.2389f }, { "ZRH", 47.4647f, 8.5492f },
  { "MUC", 48.3538f, 11.7861f }, { "VIE", 48.1103f, 16.5697f },
  { "CPH", 55.6180f, 12.6560f }, { "ARN", 59.6519f, 17.9186f },
  { "OSL", 60.1939f, 11.1004f }, { "HEL", 60.3172f, 24.9633f },
  { "DUB", 53.4213f, -6.2701f }, { "BCN", 41.2974f, 2.0833f },
  { "LIS", 38.7742f, -9.1342f }, { "ATH", 37.9364f, 23.9445f },
  { "PRG", 50.1008f, 14.2600f }, { "WAW", 52.1657f, 20.9671f },
  { "BUD", 47.4298f, 19.2611f }, { "IST", 41.2753f, 28.7519f },
  // Middle East / Asia-Pacific / Africa
  { "DXB", 25.2532f, 55.3657f }, { "DOH", 25.2731f, 51.6081f },
  { "DEL", 28.5562f, 77.1000f }, { "BOM", 19.0896f, 72.8656f },
  { "SIN", 1.3644f, 103.9915f }, { "BKK", 13.6900f, 100.7501f },
  { "KUL", 2.7456f, 101.7099f }, { "CGK", -6.1256f, 106.6558f },
  { "MNL", 14.5086f, 121.0194f }, { "SYD", -33.9399f, 151.1753f },
  { "MEL", -37.6690f, 144.8410f }, { "AKL", -37.0082f, 174.7850f },
  { "JNB", -26.1367f, 28.2411f }, { "CAI", 30.1219f, 31.4056f },
};
static const uint8_t AIRPORT_COUNT = sizeof(AIRPORTS) / sizeof(AIRPORTS[0]);

// Trail classification: is an aircraft taking off / landing at a nearby
// airport, or just flying over? Heuristic — low altitude + close to a known
// airport + significant vertical rate. Kept simple on purpose: adsb.lol
// gives no flight-plan phase, and route lookups are only opportunistically
// cached, so vertical-rate-vs-airport-proximity is the most consistent
// signal available on every blip. Thresholds are runtime-configurable via
// the web UI (classNearKm / classMaxAltFt / classVrateFpm).
// Flight-path trace history (Radar screen, showTraces toggle). Keyed by
// callsign rather than blips[] array index, since that index isn't stable
// between fetch cycles — a trail must follow one aircraft, not one array
// slot. Lives here (not shared.h/data.cpp) since it's purely a rendering
// concern local to this board's Radar screen.
// 1800 samples ~= 15-30 min of history at this screen's 1-2Hz full-redraw
// sampling rate -- long enough that no realistic single viewing session
// should ever see a trail age out by hitting this cap; only true staleness
// (the trailStaleSec config, aircraft actually gone) should ever clear a trail. Backed
// by PSRAM (ps_malloc, lazily allocated per slot and kept for the life of
// the program) rather than a fixed struct member -- 1800*16 bytes*20 slots
// is ~560KB, too big to want living in internal SRAM/.bss alongside
// everything else, but trivial against this board's 8MB PSRAM.
#define TRAIL_LEN 1800
#define TRAIL_SLOTS MAX_BLIPS
struct TrailHistory {
  char callsign[10] = {0};
  double *lat = nullptr, *lon = nullptr;  // ps_malloc'd on first use, kept across resets
  // Polar cache (distance + bearing sin/cos vs home), computed ONCE per
  // sample at append time. Projection used to be recomputed for every
  // sample on every 500ms redraw — 1800 samples x N trails of software
  // double-trig, hundreds of ms per frame. distKm/sinB/cosB are range- and
  // layout-independent, so a cached sample never needs recomputing unless
  // home location itself changes (see cacheHomeLat/Lon below).
  float *distKm = nullptr, *sinB = nullptr, *cosB = nullptr;
  uint16_t count = 0;  // TRAIL_LEN > 255, so this can't be uint8_t
  uint32_t lastSeenMs = 0;
  // Clears the trail's identity/content but keeps its PSRAM buffer
  // allocated for reuse -- a plain `t = TrailHistory()` would instead null
  // out lat/lon and leak the previous allocation.
  void reset() { callsign[0] = 0; count = 0; lastSeenMs = 0; }
  void ensureBuf() {
    if (!lat) {
      lat = (double *)ps_malloc(sizeof(double) * TRAIL_LEN);
      lon = (double *)ps_malloc(sizeof(double) * TRAIL_LEN);
      distKm = (float *)ps_malloc(sizeof(float) * TRAIL_LEN);
      sinB = (float *)ps_malloc(sizeof(float) * TRAIL_LEN);
      cosB = (float *)ps_malloc(sizeof(float) * TRAIL_LEN);
    }
  }
};
static TrailHistory trails[TRAIL_SLOTS];
// Home location the polar cache was computed against; when it differs from
// the current config, every cached sample is reprojected once (rare).
static double cacheHomeLat = 1e9, cacheHomeLon = 1e9;

enum TrailClass : uint8_t { TC_OVER = 0, TC_DEP, TC_ARR };
static TrailClass classifyBlip(const Blip &b, float nearKm, uint16_t maxAltFt, int16_t vrateFpm) {  if (b.altitudeM * 3.28084f >= (float)maxAltFt) return TC_OVER;
  for (uint8_t a = 0; a < AIRPORT_COUNT; a++) {
    if (haversineKm(b.lat, b.lon, AIRPORTS[a].lat, AIRPORTS[a].lon) <= (double)nearKm) {
      if (b.vrateFpm > vrateFpm) return TC_DEP;
      if (b.vrateFpm < -vrateFpm) return TC_ARR;
      return TC_OVER;
    }
  }
  return TC_OVER;
}
static const Blip *findBlipByCallsign(const char *callsign) {
  for (uint8_t i = 0; i < blipCount; i++)
    if (strcmp(blips[i].callsign, callsign) == 0) return &blips[i];
  return nullptr;
}

// Float-precision polar projection for the trail rendering loop. Double-
// precision trig is software-emulated on the ESP32-S3 (~20-50us per call),
// and projecting 1800 samples x several trails with the double haversineKm/
// bearingDeg helpers on every full redraw cost 450-850ms per frame (measured
// via /timingdebug — it looked like a panel problem but was CPU). Float trig
// is ~10-20x faster and accurate to ~1m at radar scale — sub-pixel.
static inline void trailPolar(float lat1, float lon1, float lat2, float lon2, float &distKm, float &brgDeg) {
  const float D2R = (float)PI / 180.0f;
  float dLat = (lat2 - lat1) * D2R, dLon = (lon2 - lon1) * D2R;
  float s1 = sinf(dLat * 0.5f), s2 = sinf(dLon * 0.5f);
  float a = s1 * s1 + cosf(lat1 * D2R) * cosf(lat2 * D2R) * s2 * s2;
  distKm = 6371.0f * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
  float y = sinf(dLon) * cosf(lat2 * D2R);
  float x = cosf(lat1 * D2R) * sinf(lat2 * D2R) - sinf(lat1 * D2R) * cosf(lat2 * D2R) * cosf(dLon);
  brgDeg = fmodf(atan2f(y, x) / D2R + 360.0f, 360.0f);
}

// Projects one trail sample into the polar cache (distKm/sinB/cosB) —
// called exactly once per sample (at append, or when home location changes),
// never in the per-redraw draw loop.
static void trailProject(TrailHistory &t, uint16_t idx, double hLat, double hLon) {
  float dist, brg;
  trailPolar((float)hLat, (float)hLon, (float)t.lat[idx], (float)t.lon[idx], dist, brg);
  float br = brg * (float)PI / 180.0f;
  t.distKm[idx] = dist;
  t.sinB[idx] = sinf(br);
  t.cosB[idx] = cosf(br);
}

// Appends one sample, keeping lat/lon and the polar cache in sync. Shift-on-
// full uses memmove for all five arrays (the old per-element loop moved
// 28.8KB by hand every append).
static void trailAppend(TrailHistory &t, uint16_t cap, double hLat, double hLon, double blat, double blon) {
  if (t.count < cap) {
    t.lat[t.count] = blat; t.lon[t.count] = blon;
    trailProject(t, t.count, hLat, hLon);
    t.count++;
  } else {
    memmove(t.lat, t.lat + 1, (cap - 1) * sizeof(double));
    memmove(t.lon, t.lon + 1, (cap - 1) * sizeof(double));
    memmove(t.distKm, t.distKm + 1, (cap - 1) * sizeof(float));
    memmove(t.sinB, t.sinB + 1, (cap - 1) * sizeof(float));
    memmove(t.cosB, t.cosB + 1, (cap - 1) * sizeof(float));
    t.lat[cap - 1] = blat; t.lon[cap - 1] = blon;
    trailProject(t, cap - 1, hLat, hLon);
  }
}

// Real double-buffered RGB panel, bypassing Arduino_GFX's RGB bus classes
// entirely — confirmed by reading Arduino_ESP32RGBPanel's source directly
// (both the 1.4.5 this project pins for the other boards, and the current
// 1.6.7) that it has never implemented true double buffering, at any
// version: "It uses a Single Frame Buffer in PSRAM" per the class's own
// header comment. bounce_buffer_size_px exists but only helps PSRAM
// bandwidth contention (already solved via pclk=16MHz on [env:lcd7b]) — not
// draw-tearing. True double buffering needs num_fbs=2 at the ESP-IDF layer,
// which doesn't exist in the older ESP-IDF this project's other boards'
// core (2.0.14) bundles — hence this being a separate core-3.x environment.
//
// Pin/timing values below are unchanged from [env:lcd7b]'s already-verified
// config (same pins, same porches, same pclk=16MHz, same polarity=0/0) —
// only the buffering strategy differs.
static esp_lcd_panel_handle_t panelHandle = NULL;

// The driver's two framebuffers (num_fbs=2, allocated by esp_lcd itself in
// PSRAM, fetched in initRGBPanel()) and which one the canvas currently
// targets. Declared up here because initRGBPanel() populates them.
static uint16_t *panelFbs[2] = {nullptr, nullptr};
static uint8_t drawBufIdx = 0;

// Staged-redraw state (declared early so /health in displaySetup can read
// them). See the staging section near renderRadar for how these are used.
enum StageSlice : uint8_t {
  SL_IDLE = 0,
  SL_FILL_A, SL_FILL_B, SL_FILL_C, SL_FILL_D,
  SL_STATIC,
  SL_TRAILS_A, SL_TRAILS_B, SL_TRAILS_C, SL_TRAILS_D,
  SL_BLIPS,
  SL_COPY_A, SL_COPY_B, SL_COPY_C, SL_COPY_D, SL_COPY_E, SL_COPY_F, SL_COPY_G, SL_COPY_H,
  SL_SWAP,
  SL_SYNC_A, SL_SYNC_B, SL_SYNC_C, SL_SYNC_D, SL_SYNC_E, SL_SYNC_F, SL_SYNC_G, SL_SYNC_H,
};
static StageSlice slice = SL_IDLE;
static uint32_t lastFullCycle = 0;
#define GLOW_MAX 12
static int glowX[2][GLOW_MAX], glowY[2][GLOW_MAX];
static uint8_t glowCount[2] = {0, 0};

// Touch diagnostics for /health — last raw touch coords + what the handler
// did with them (button hit vs screen cycle).
static volatile int dbgTouchX = -1, dbgTouchY = -1;
static char dbgTouchAction[24] = "none";

// Fires once the hardware has genuinely finished consuming the frame buffer
// content we last handed it via draw_bitmap -- confirmed via Waveshare's own
// LVGL reference for this exact panel (rgb_lcd_port.cpp/lvgl_port.cpp):
// their flush callback calls draw_bitmap, then blocks on this exact
// notification (on_bounce_frame_finish, aliased to on_frame_buf_complete)
// before letting LVGL render the next frame. Our own pushFrame()/
// pushPartial() were firing draw_bitmap with no wait at all (just a 1-tick
// vTaskDelay) -- nothing stopped us writing into a buffer the DMA scanner
// was still mid-read on, which is the actual cause of the ghosting/tearing
// that survived every earlier fix (full-vs-partial push, refresh_on_demand,
// the stale-sweep-position bug). This callback runs continuously at the
// panel's own refresh rate regardless of whether we drew anything, so
// waitFrameDone() below drains any stale pending signal first (mirroring
// Waveshare's own ulTaskNotifyValueClear() immediately before their wait) --
// otherwise we could consume a leftover signal from before our draw call
// and return without actually having waited for it.
static SemaphoreHandle_t frameDoneSem = nullptr;
IRAM_ATTR static bool onFrameBufComplete(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx) {
  BaseType_t hpw = pdFALSE;
  xSemaphoreGiveFromISR(frameDoneSem, &hpw);
  return hpw == pdTRUE;
}
// Incremented every VSYNC (panel scan-out start). Direct (active-buffer)
// sweep writes wait on this and land in the vblank gap (~4ms), so they never
// split the line mid-scan — arbitrary-timing active-fb writes were the
// visible "jittery sweep" artifact.
static volatile uint32_t vsyncCount = 0;
IRAM_ATTR static bool onVsync(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx) {
  vsyncCount++;
  return false;
}
static void waitFrameDone() {
  while (xSemaphoreTake(frameDoneSem, 0) == pdTRUE) {}  // drain stale signal
  xSemaphoreTake(frameDoneSem, pdMS_TO_TICKS(200));
}

void initRGBPanel() {
  esp_lcd_rgb_panel_config_t cfg = {};
  cfg.clk_src = LCD_CLK_SRC_DEFAULT;
  // pclk stepped in ISOLATION (only change in the build) per the debug-log
  // rule. Physical refresh = pclk / ~(1386x661 total px) — this is the hard
  // ceiling on sweep smoothness. Espressif's tested ceilings (ESP-FAQ, LCD
  // section): ~22MHz max with octal PSRAM @80MHz (our config); 30MHz needs
  // PSRAM @120MHz + flash @120MHz (custom sdkconfig rebuild — not done).
  // Steps verified on hardware, watching for the horizontal-drift symptom
  // (bounce-buffer underrun) at each: 10MHz ok (zero-copy build) ->
  // 16MHz ok -> 20MHz ok -> 22MHz UNSTABLE (drift — right at Espressif's
  // ~22MHz tested ceiling for octal PSRAM @80MHz, no margin).
  // 30MHz requires the 120MHz-PSRAM build config, which lives in
  // [env:lcd7b_v3] (USB-flash-only — see platformio.ini). Set per-env via
  // LCD7B_PCLK_HZ so a stable v2 build can never accidentally carry 30MHz.
  #ifndef LCD7B_PCLK_HZ
  #define LCD7B_PCLK_HZ 20000000
  #endif
  cfg.timings.pclk_hz = LCD7B_PCLK_HZ;
  cfg.timings.h_res = LCD_W;
  cfg.timings.v_res = LCD_H;
  cfg.timings.hsync_pulse_width = 162;
  cfg.timings.hsync_back_porch = 152;
  cfg.timings.hsync_front_porch = 48;
  cfg.timings.vsync_pulse_width = 45;
  cfg.timings.vsync_back_porch = 13;
  cfg.timings.vsync_front_porch = 3;
  cfg.timings.flags.pclk_active_neg = 1;
  cfg.data_width = 16;
  cfg.bits_per_pixel = 16;
  cfg.num_fbs = 2;  // the actual fix — two hardware-managed buffers instead of one
  // dma_burst_size replaces the old sram_trans_align/psram_trans_align pair
  // (deprecated, now a union with this). 64 matches what Arduino_ESP32RGBPanel
  // explicitly set for psram_trans_align in the old single-buffer path.
  cfg.dma_burst_size = 64;
  // Bounce buffer: this is the real fix for the "screen drift"/horizontal-
  // shift symptom actually hit on hardware. Root cause per Espressif's own
  // RGB-panel troubleshooting notes: GDMA can hit a FIFO under-run reading
  // frame data straight from PSRAM, after which the LCD controller keeps
  // pulling pixels from the wrong address — a line-by-line shift. Their
  // documented fix is enabling the bounce buffer (fast internal-SRAM staging
  // buffers DMA reads from instead of PSRAM directly), sized >= 20 lines to
  // keep enough VBlank margin for the ISR refill. This is also why this had
  // to be a core-3.x environment in the first place, separate from that
  // research: this fix needs ESP-IDF >= 5.1, which arduino-esp32 2.x (this
  // project's other boards) bundles an older ESP-IDF that predates.
  cfg.bounce_buffer_size_px = LCD_W * 20;
  cfg.hsync_gpio_num = PIN_HSYNC;
  cfg.vsync_gpio_num = PIN_VSYNC;
  cfg.de_gpio_num = PIN_DE;
  cfg.pclk_gpio_num = PIN_PCLK;
  cfg.disp_gpio_num = -1;
  // Same non-bigEndian data line order Arduino_ESP32RGBPanel used: B0-4,
  // G0-5, R0-4 (indices 0-15), matching this board's official pin mapping.
  cfg.data_gpio_nums[0] = PIN_B0; cfg.data_gpio_nums[1] = PIN_B1; cfg.data_gpio_nums[2] = PIN_B2;
  cfg.data_gpio_nums[3] = PIN_B3; cfg.data_gpio_nums[4] = PIN_B4;
  cfg.data_gpio_nums[5] = PIN_G0; cfg.data_gpio_nums[6] = PIN_G1; cfg.data_gpio_nums[7] = PIN_G2;
  cfg.data_gpio_nums[8] = PIN_G3; cfg.data_gpio_nums[9] = PIN_G4; cfg.data_gpio_nums[10] = PIN_G5;
  cfg.data_gpio_nums[11] = PIN_R0; cfg.data_gpio_nums[12] = PIN_R1; cfg.data_gpio_nums[13] = PIN_R2;
  cfg.data_gpio_nums[14] = PIN_R3; cfg.data_gpio_nums[15] = PIN_R4;
  cfg.flags.fb_in_psram = 1;
  // Tried refresh_on_demand=1 here to fix a "two lines, one paused one
  // moving" ghosting artifact (theory: continuous-stream mode's autonomous
  // background re-scan racing our buffer flips). Result was a fully black
  // screen instead -- this "dumb" RGB panel has no onboard memory of its
  // own (unlike the JC4832W535's QSPI panel), so it needs a genuinely
  // continuous signal just to show anything at all; refresh_on_demand
  // apparently stops that signal between our (infrequent, ~30-150ms
  // spaced) triggers rather than just gating when new content transmits.
  // Reverted -- default continuous-stream mode, ghosting not yet solved.

  ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&cfg, &panelHandle));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panelHandle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panelHandle));

  frameDoneSem = xSemaphoreCreateBinary();
  esp_lcd_rgb_panel_event_callbacks_t cbs = {};
  cbs.on_frame_buf_complete = onFrameBufComplete;
  cbs.on_vsync = onVsync;
  ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panelHandle, &cbs, NULL));

  // Fetch the driver's own two framebuffers so the canvas can draw directly
  // into them (zero-copy present — see DirectCanvas/pushFrame above).
  void *fb0 = nullptr, *fb1 = nullptr;
  ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panelHandle, 2, &fb0, &fb1));
  panelFbs[0] = (uint16_t *)fb0;
  panelFbs[1] = (uint16_t *)fb1;
}

// Zero-copy rendering surface. Arduino_Canvas allocates its own framebuffer
// in begin() only if _framebuffer is still null, so this subclass injects a
// pointer BEFORE begin() — and then the canvas rasterizes straight into one
// of the panel driver's own two framebuffers, with no buffer of its own.
// (The destructor would free() the injected pointer, but this object lives
// forever, so it never runs.)
class DirectCanvas : public Arduino_Canvas {
public:
  DirectCanvas(int16_t w, int16_t h) : Arduino_Canvas(w, h, nullptr) {}
  void setBuffer(uint16_t *buf) { _framebuffer = buf; }
};
DirectCanvas *directGfx = new DirectCanvas(LCD_W, LCD_H);

// The global rendering target. Points at directGfx (one of the panel
// framebuffers) except while a staged redraw is rendering into the staging
// canvas — all screen code draws through this, unaware of the retargeting.
Arduino_GFX *gfx = directGfx;

// Third PSRAM framebuffers: the staged radar redraw renders the next frame
// into one of them in small slices between sweep ticks (ping-pong), then
// it's band-copied into the panel framebuffers (see renderRadar). The other
// always holds the last COMPLETED frame (bgFrame) and serves as the clean
// background source for sweep-line erases — see restoreLine().
static Arduino_Canvas *stagingCanvas[2] = { nullptr, nullptr };
static uint8_t stgRenderIdx = 0;              // which staging buffer the current cycle renders into
static Arduino_Canvas *bgFrame = nullptr;     // last completed full frame (valid background source)

// Erases a line by RESTORING the background under it from a source frame
// buffer instead of painting BLACK. Erasing to black cut a 1px gash through
// any label/ring/trail the sweep crossed, which accumulated into visibly
// "fuzzy" plane names between full redraws (sweep stayed clean — the damage
// was to the text). Bresenham over raw RGB565 buffers.
//
// plotLine() below draws the sweep with THE EXACT SAME rasterization —
// drawing with Arduino_GFX's drawLine but erasing with this Bresenham left
// un-erased specks along the old line (the two algorithms' tie-breaking
// differs by a pixel here and there), which showed up as sweep artifacts.
static inline void linePixels(uint16_t *dst, const uint16_t *src, int x0, int y0, int x1, int y1, uint16_t color) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    if (x0 >= 0 && x0 < LCD_W && y0 >= 0 && y0 < LCD_H)
      dst[(size_t)y0 * LCD_W + x0] = src ? src[(size_t)y0 * LCD_W + x0] : color;
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}
static void restoreLine(uint16_t *dst, const uint16_t *src, int x0, int y0, int x1, int y1) {
  linePixels(dst, src, x0, y0, x1, y1, 0);
}
static void plotLine(uint16_t *dst, int x0, int y0, int x1, int y1, uint16_t color) {
  linePixels(dst, nullptr, x0, y0, x1, y1, color);
}

// Presents the just-drawn frame: hands the driver the pointer of the buffer
// the canvas drew into. Verified against esp_lcd_panel_rgb.c @ v5.5.5 (the
// exact IDF this build bundles): when the color_data pointer falls inside
// one of the driver's own framebuffers, draw_bitmap() does NO copy at all —
// it just sets cur_fb_index to that buffer (adopted by the bounce engine at
// the next frame wrap) and cache-syncs. The previous design instead kept a
// separate 1.2MB canvas and CPU-copied the whole thing into the driver fb on
// every frame — doubling the cost of every frame and putting a 1.2MB
// PSRAM->PSRAM memcpy on the exact same bus the DMA scan-out reads from,
// which is precisely the contention that forces pclk (and therefore the
// panel's physical refresh rate) down. waitFrameDone() then blocks until the
// hardware has fully streamed the OTHER buffer, making it safe to draw into
// next — drawing into a buffer mid-scan is what tearing looks like.
void pushFrame() {
  esp_lcd_panel_draw_bitmap(panelHandle, 0, 0, LCD_W, LCD_H, panelFbs[drawBufIdx]);
  waitFrameDone();
  drawBufIdx ^= 1;
  directGfx->setBuffer(panelFbs[drawBufIdx]);
}

IOExtension ioExpander;
GT911_Touch touch(TOUCH_INT, ioExpander);

#define LCD7B_NUM_SCREENS 5  // Target Intel, Top 5, Radar, Weather & System, Radar Full

// Per-frame timing breakdown, updated every render() call, read via
// GET /timingdebug -- so we can see exactly where a frame's time goes
// (canvas clear vs. screen draw vs. driver push) instead of guessing.
static volatile uint32_t lastFillUs = 0, lastDrawUs = 0, lastPushUs = 0, lastFrameIntervalMs = 0;

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

// Diamond marker for airports — distinct shape AND color (configurable,
// MAGENTA by default) so it can't be confused with blips, traces, rings,
// or the sweep.
static void drawAirportMarker(int px, int py, uint16_t color) {
  gfx->drawLine(px, py - 6, px + 6, py, color);
  gfx->drawLine(px + 6, py, px, py + 6, color);
  gfx->drawLine(px, py + 6, px - 6, py, color);
  gfx->drawLine(px - 6, py, px, py - 6, color);
}

// On-screen key explaining the trace classification colors + airport
// marker. Drawn only when showTraces is on (no traces, nothing to explain).
static void drawTrailKey(int x, int y, uint16_t cDep, uint16_t cArr, uint16_t cOver, uint16_t cApt) {
  const char *labels[4] = { "TAKEOFF", "LANDING", "FLYOVER", "AIRPORT" };
  const uint16_t colors[4] = { cDep, cArr, cOver, cApt };
  gfx->setTextSize(2);
  for (uint8_t i = 0; i < 4; i++) {
    if (i == 3) drawAirportMarker(x + 10, y + 8, colors[i]);
    else gfx->drawFastHLine(x, y + 8, 20, colors[i]);
    gfx->setTextColor(colors[i], BLACK);
    gfx->setCursor(x + 30, y);
    gfx->print(labels[i]);
    y += 32;
  }
}

// Places a multi-line blip label near (bx,by), avoiding overlap with every
// label already placed this frame. All label lines use textSize(2); the
// caller must set that before calling.
struct LabelRect { int x, y, w, h; };

bool placeLabel(LabelRect *placed, int &placedCount, int maxPlaced,
                 int bx, int by, char lines[][12], int lineCount,
                 int boundX, int boundY, int boundW, int boundH, uint16_t color) {
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
      gfx->setTextColor(color, BLACK);
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
// Full-screen radar's bigger set, bottom-left margin.
static const int FRNG_BTN_X = 20, FRNG_BTN_W = 195, FRNG_BTN_H = 85;
static const int FRNG_PLUS_Y = 405, FRNG_MINUS_Y = 500;

// The sweep-line endpoint screenRadar() ACTUALLY drew on its last full
// redraw, recorded at draw time. renderRadar() seeds its per-buffer erase
// position from this — recomputing the angle from a fresh millis() after the
// (slow) draw would produce a position that was never on screen, which was
// the original "one paused line, one moving line" ghosting bug.
static int drawnSweepX = RADAR_CX, drawnSweepY = RADAR_CY - RADAR_R;

// Reads the radar-relevant config under configMutex into a snapshot struct,
// shared by both radar screens.
struct RadarCfg {
  double hLat, hLon; float rMax;
  bool showCS, showAir, showSpd, showFlt, showRte, showRg, showSq, showVr, showTy;
  uint8_t maxBlips; uint16_t trailSamp, trailStale, cAlt, redrawMs;
  float swpSec, cNear; int16_t cVr;
  bool sApt, sKey, sComp, quiet;
  uint16_t cSweep, cBlip, cBlipHi, cRings, cAirpt, cTrDep, cTrArr, cTrOver;
  FilterRule rules[FILTER_MAX_RULES];
};
static RadarCfg radarCfgSnapshot() {
  RadarCfg c;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  c.hLat = homeLat; c.hLon = homeLon; c.rMax = radarMaxKm;
  c.showCS = showCallsign; c.showAir = showAirline; c.showSpd = showSpeed; c.showFlt = showFL; c.showRte = showRoute;
  c.showRg = showReg; c.showSq = showSquawk; c.showVr = showVRate; c.showTy = showType;
  c.maxBlips = maxBlipsShown; c.trailSamp = trailMaxSamples; c.trailStale = trailStaleSec;
  c.cNear = classNearKm; c.cAlt = classMaxAltFt; c.cVr = classVrateFpm;
  c.swpSec = sweepPeriodSec; c.redrawMs = radarRedrawMs;
  c.sApt = showAirports; c.sKey = showTrailKey; c.sComp = showCompass;
  c.cSweep = colSweep; c.cBlip = colBlip; c.cBlipHi = colBlipHi; c.cRings = colRings; c.cAirpt = colAirport;
  c.cTrDep = colTrailDep; c.cTrArr = colTrailArr; c.cTrOver = colTrailOver;
  c.quiet = filterQuiet;
  memcpy(c.rules, filterRules, sizeof(c.rules));
  if (configMutex) xSemaphoreGive(configMutex);
  return c;
}

// Index of the first filter rule matching this blip, or -1.
static int filterMatchBlip(const Blip &b, const FilterRule *rules) {
  for (uint8_t i = 0; i < FILTER_MAX_RULES; i++) {
    if (!rules[i].enabled || !rules[i].text[0]) continue;
    const char *field = rules[i].match == FM_REG ? b.reg : rules[i].match == FM_TYPE ? b.typeCode : b.callsign;
    if (filterPrefixMatch(field, rules[i].text)) return i;
  }
  return -1;
}
void drawRadarFurniture(const RadarLayout &L, const RadarCfg &cfg);
void drawRadarStatic(const RadarLayout &L, const RadarCfg &cfg);
void drawRadarTrails(const RadarLayout &L, const RadarCfg &cfg, uint8_t slotFrom, uint8_t slotTo);
void drawRadarBlips(const RadarLayout &L, const RadarCfg &cfg);

// Full radar frame, used by the monolithic redraw path (screen entry).
void drawRadarAll(const RadarLayout &L) {
  RadarCfg cfg = radarCfgSnapshot();
  drawRadarFurniture(L, cfg);
  drawRadarStatic(L, cfg);
  drawRadarTrails(L, cfg, 0, TRAIL_SLOTS);
  drawRadarBlips(L, cfg);
}

void screenRadar() { drawRadarAll(radarLayout(2)); }
void screenRadarFull() { drawRadarAll(radarLayout(4)); }

// Screen furniture around the radar circle: header + left info column +
// zoom buttons (classic layout), or corner readouts + big zoom buttons
// (full-screen layout).
// Animated weather widget (~44x36px): sun with slowly rotating rays by day,
// crescent moon + twinkling stars by night, drifting clouds, falling rain,
// flashing storm. All motion is driven by a 2Hz frame counter — the widget
// is furniture inside the staged redraw, so the animation is free (no
// per-frame work).
void drawWeatherWidget(int x, int y, int code) {
  uint32_t f = millis() / 500;
  bool night = isNightNow();
  bool cloudy = (code >= 1 && code <= 3) || code == 45 || code == 48 ||
                (code >= 51 && code <= 67) || (code >= 71 && code <= 77) ||
                (code >= 80 && code <= 82) || code == 85 || code == 86 || code >= 95;
  bool rain = (code >= 51 && code <= 67) || (code >= 80 && code <= 82);
  bool snow = (code >= 71 && code <= 77) || code == 85 || code == 86;
  bool storm = code >= 95;

  if (!cloudy || code <= 3) {
    if (night) {
      // Crescent moon (pale circle with offset cutout) + blinking stars
      gfx->fillCircle(x + 14, y + 12, 8, LIGHTGREY);
      gfx->fillCircle(x + 18, y + 10, 7, BLACK);
      if (f % 2 == 0) {
        gfx->drawPixel(x + 30, y + 4, WHITE);
        gfx->drawPixel(x + 36, y + 12, WHITE);
      } else {
        gfx->drawPixel(x + 32, y + 8, WHITE);
      }
    } else {
      // Sun with rays rotating ~15deg per cycle
      int sx = x + 16, sy = y + 12;
      gfx->fillCircle(sx, sy, 7, YELLOW);
      float a0 = (f % 24) * (2.0f * (float)PI / 24.0f);
      for (int i = 0; i < 8; i++) {
        float a = a0 + i * (float)PI / 4.0f;
        gfx->drawLine(sx + (int)(sinf(a) * 10), sy - (int)(cosf(a) * 10),
                      sx + (int)(sinf(a) * 14), sy - (int)(cosf(a) * 14), YELLOW);
      }
    }
  }

  if (code == 45 || code == 48) {  // fog
    for (int i = 0; i < 3; i++)
      gfx->drawFastHLine(x + 4 + (i == (int)(f % 3) ? 3 : 0), y + 14 + i * 5, 26, LIGHTGREY);
  } else if (cloudy) {
    int drift = (int)(sinf((f % 40) * (2.0f * (float)PI / 40.0f)) * 3);
    drawCloudShape(x + 8 + drift, y + 12, LIGHTGREY);
  }

  if (rain) {
    for (int i = 0; i < 3; i++) {
      int dy = (int)((f + i) % 3) * 4;
      gfx->drawLine(x + 12 + i * 7, y + 24 + dy, x + 10 + i * 7, y + 28 + dy, CYAN);
    }
  } else if (snow) {
    for (int i = 0; i < 3; i++) {
      int dx = (int)(f % 2) * 2;
      int sx2 = x + 12 + i * 7 + dx, sy2 = y + 25 + (i % 2) * 3;
      gfx->drawFastHLine(sx2 - 2, sy2, 5, WHITE);
      gfx->drawFastVLine(sx2, sy2 - 2, 5, WHITE);
    }
  } else if (storm && f % 2 == 0) {  // bolt flashes on alternate cycles
    gfx->drawLine(x + 18, y + 22, x + 14, y + 30, YELLOW);
    gfx->drawLine(x + 14, y + 30, x + 20, y + 30, YELLOW);
    gfx->drawLine(x + 20, y + 30, x + 15, y + 38, YELLOW);
  }
}

void drawRadarFurniture(const RadarLayout &L, const RadarCfg &cfg) {
  if (L.full) {
    // Weather, top-left corner
    gfx->setTextSize(2);
    if (weather.valid) {
      drawWeatherWidget(10, 8, weather.code);
      gfx->setTextColor(CYAN, BLACK);
      gfx->setCursor(50, 16);
      gfx->printf("%.1fC   ", weather.tempC);
    }

    // Contacts + nearest, top-right corner
    gfx->setTextColor(WHITE, BLACK);
    gfx->setCursor(830, 12);
    gfx->printf("Contacts: %u   ", blipCount);
    gfx->setTextColor(CYAN, BLACK);
    gfx->setCursor(830, 40);
    if (nearest.valid) gfx->printf("%-11s", nearest.callsign);
    else gfx->print("No TGT     ");

    // Range + big zoom buttons, bottom-left margin
    gfx->setTextSize(3);
    gfx->setTextColor(CYAN, BLACK);
    gfx->setCursor(20, 350);
    gfx->printf("RNG %d km   ", (int)cfg.rMax);

    gfx->fillRoundRect(FRNG_BTN_X, FRNG_PLUS_Y, FRNG_BTN_W, FRNG_BTN_H, 8, DARKGREY);
    gfx->setTextColor(WHITE, DARKGREY);
    gfx->setCursor(FRNG_BTN_X + 85, FRNG_PLUS_Y + 25);
    gfx->print("+");

    gfx->fillRoundRect(FRNG_BTN_X, FRNG_MINUS_Y, FRNG_BTN_W, FRNG_BTN_H, 8, DARKGREY);
    gfx->setCursor(FRNG_BTN_X + 85, FRNG_MINUS_Y + 25);
    gfx->print("-");
    return;
  }

  drawHeader("RADAR");

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
    drawWeatherWidget(15, 185, weather.code);
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
  gfx->printf("RNG %d km   ", (int)cfg.rMax);

  gfx->fillRoundRect(RNG_BTN_X, RNG_PLUS_Y, RNG_BTN_W, RNG_BTN_H, 6, DARKGREY);
  gfx->setTextColor(WHITE, DARKGREY);
  gfx->setCursor(RNG_BTN_X + 75, RNG_PLUS_Y + 18);
  gfx->print("+");

  gfx->fillRoundRect(RNG_BTN_X, RNG_MINUS_Y, RNG_BTN_W, RNG_BTN_H, 6, DARKGREY);
  gfx->setCursor(RNG_BTN_X + 75, RNG_MINUS_Y + 18);
  gfx->print("-");
}

// Rings/axes, compass, airports, sweep line (records drawnSweepX/Y), and
// trail sampling. Split from drawRadarTrails/drawRadarBlips so the staged
// redraw (renderRadar) can render each part in its own time slice.
void drawRadarStatic(const RadarLayout &L, const RadarCfg &cfg) {
  const int cx = L.cx, cy = L.cy, R = L.R;
  uint32_t nowMs = millis();
  float sweepDeg = fmodf(millis() / (cfg.swpSec * 1000.0f / 360.0f), 360.0f);
  double sw = deg2rad(sweepDeg);

  gfx->drawCircle(cx, cy, R, cfg.cRings);
  gfx->drawCircle(cx, cy, R * 2 / 3, cfg.cRings);
  gfx->drawCircle(cx, cy, R / 3, cfg.cRings);
  gfx->drawLine(cx - R, cy, cx + R, cy, cfg.cRings);
  gfx->drawLine(cx, cy - R, cx, cy + R, cfg.cRings);

  // Compass — matches the same north-up, clockwise convention already
  // used for blip placement (bx = cx + sin(brg)*r, by = cy - cos(brg)*r,
  // brg 0=N/90=E/180=S/270=W), so these letters are correct relative to
  // where blips actually land, not just decorative. Classic layout puts
  // them OUTSIDE the circle (header/column leave margin); the full-screen
  // layout's circle nearly touches the canvas edges, so they go INSIDE.
  if (cfg.sComp) {
    gfx->setTextSize(2);
    gfx->setTextColor(LIGHTGREY, BLACK);
    int16_t lx1, ly1; uint16_t lw, lh;
    gfx->getTextBounds("N", 0, 0, &lx1, &ly1, &lw, &lh);
    if (L.full) {
      gfx->setCursor(cx - lw / 2, cy - R + 6); gfx->print("N");
      gfx->setCursor(cx - lw / 2, cy + R - lh - 6); gfx->print("S");
      gfx->getTextBounds("E", 0, 0, &lx1, &ly1, &lw, &lh);
      gfx->setCursor(cx + R - lw - 8, cy - lh / 2); gfx->print("E");
      gfx->getTextBounds("W", 0, 0, &lx1, &ly1, &lw, &lh);
      gfx->setCursor(cx - R + 8, cy - lh / 2); gfx->print("W");
    } else {
      gfx->setCursor(cx - lw / 2, cy - R - lh - 6); gfx->print("N");
      gfx->setCursor(cx - lw / 2, cy + R + 6); gfx->print("S");
      gfx->getTextBounds("E", 0, 0, &lx1, &ly1, &lw, &lh);
      gfx->setCursor(cx + R + 8, cy - lh / 2); gfx->print("E");
      gfx->getTextBounds("W", 0, 0, &lx1, &ly1, &lw, &lh);
      gfx->setCursor(cx - R - 8 - lw, cy - lh / 2); gfx->print("W");
    }
  }

  // --- Physical airports in range: diamond marker + IATA code ---
  if (cfg.sApt) {
    gfx->setTextColor(cfg.cAirpt, BLACK);
    for (uint8_t a = 0; a < AIRPORT_COUNT; a++) {
      double dist = haversineKm(cfg.hLat, cfg.hLon, AIRPORTS[a].lat, AIRPORTS[a].lon);
      float fr = (float)(dist / cfg.rMax);
      if (fr > 1) continue;
      int rr = (int)(fr * R);
      double brg = bearingDeg(cfg.hLat, cfg.hLon, AIRPORTS[a].lat, AIRPORTS[a].lon);
      int px = cx + (int)(sin(deg2rad(brg)) * rr);
      int py = cy - (int)(cos(deg2rad(brg)) * rr);
      drawAirportMarker(px, py, cfg.cAirpt);
      gfx->setCursor(px + 9, py - 8);
      gfx->print(AIRPORTS[a].iata);
    }
  }

  // Record where the sweep IS now — but do NOT draw it. The sweep is drawn
  // exclusively by the tick paths (fast zero-copy ticks, direct draws during
  // staging), never into a staged frame: bgFrame is the clean background
  // source for restoreLine() erases, and a sweep line baked into it would be
  // resurrected as a ghost line on every erase. The endpoint is still needed
  // to seed each buffer's erase position at SL_SWAP / screen entry.
  drawnSweepX = cx + (int)(sin(sw) * R);
  drawnSweepY = cy - (int)(cos(sw) * R);

  // --- Trail sampling: append one position per aircraft per redraw cycle.
  // Keyed by callsign since blips[] array indices aren't stable between
  // fetch cycles. Cap is runtime-configurable (trailMaxSamples).
  if (showTraces) {
    // Home moved since the polar cache was computed? Reproject everything
    // once (rare — only on a config change).
    if (cacheHomeLat != cfg.hLat || cacheHomeLon != cfg.hLon) {
      for (uint8_t s = 0; s < TRAIL_SLOTS; s++) {
        TrailHistory &t = trails[s];
        if (!t.callsign[0] || !t.lat) continue;
        for (uint16_t j = 0; j < t.count; j++) trailProject(t, j, cfg.hLat, cfg.hLon);
      }
      cacheHomeLat = cfg.hLat; cacheHomeLon = cfg.hLon;
    }
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
      // Runtime-configurable cap (trailMaxSamples), never above the
      // compiled-in buffer size. If the cap was lowered at runtime, drop the
      // oldest samples once to get back under it.
      uint16_t cap = cfg.trailSamp < TRAIL_LEN ? cfg.trailSamp : TRAIL_LEN;
      if (t.count > cap) {
        uint16_t drop = t.count - cap;
        memmove(t.lat, t.lat + drop, cap * sizeof(double));
        memmove(t.lon, t.lon + drop, cap * sizeof(double));
        memmove(t.distKm, t.distKm + drop, cap * sizeof(float));
        memmove(t.sinB, t.sinB + drop, cap * sizeof(float));
        memmove(t.cosB, t.cosB + drop, cap * sizeof(float));
        t.count = cap;
      }
      // Store the dead-reckoned position (same projection as the blip icon)
      // so the trail's tail meets the plane instead of lagging it by the
      // data age (trails used to end at the last REPORTED position while
      // the icon flew ahead — "trace lines don't catch up to the planes").
      double pla = blips[i].lat, plo = blips[i].lon;
      if (blips[i].speedMs > 0) {
        float el = (nowMs - lastDataMs) / 1000.0f;
        if (el > 0) projectLatLon(blips[i].lat, blips[i].lon, blips[i].track, blips[i].speedMs * el, pla, plo);
      }
      trailAppend(t, cap, cfg.hLat, cfg.hLon, pla, plo);
      t.lastSeenMs = nowMs;
    }
  }
}

// Trails for trail slots [slotFrom, slotTo) + the color key (on the final
// slice). Color encodes flight phase (see classifyBlip) — all three colors
// and the thresholds are configurable via the web UI. Entries not seen in
// trailStaleSec are dropped.
void drawRadarTrails(const RadarLayout &L, const RadarCfg &cfg, uint8_t slotFrom, uint8_t slotTo) {
  if (!showTraces) return;
  const int cx = L.cx, cy = L.cy, R = L.R;
  uint32_t nowMs = millis();
  const uint16_t trailColors[3] = { cfg.cTrOver, cfg.cTrDep, cfg.cTrArr };  // TC_OVER, TC_DEP, TC_ARR
  for (uint8_t s = slotFrom; s < slotTo; s++) {
    TrailHistory &t = trails[s];
    if (!t.callsign[0]) continue;
    if (nowMs - t.lastSeenMs > (uint32_t)cfg.trailStale * 1000UL) { t.reset(); continue; }
      // Connected polyline, not separate dots -- a trail of isolated 2-3px
      // dots is nearly indistinguishable from noise at this scale (typical
      // sample-to-sample movement is only a few px), and doesn't read as
      // "a path" the way a real radar trace does.
      const Blip *owner = findBlipByCallsign(t.callsign);
      uint16_t trailColor;
      if (!owner) {
        trailColor = cfg.cTrOver;
      } else {
        int rule = filterMatchBlip(*owner, cfg.rules);
        if (rule >= 0 && cfg.rules[rule].action == FA_HIDE) continue;  // hidden traffic leaves no trail
        trailColor = (rule >= 0 && (cfg.rules[rule].action == FA_HIGHLIGHT || cfg.rules[rule].action == FA_ALERT))
                     ? cfg.rules[rule].color
                     : trailColors[classifyBlip(*owner, cfg.cNear, cfg.cAlt, cfg.cVr)];
      }
      int prevPx = 0, prevPy = 0; bool havePrev = false;
      for (uint16_t j = 0; j < t.count; j++) {
        // Polar cache hit: no trig here at all, just scale + offset.
        float fr = t.distKm[j] / cfg.rMax;
        if (fr > 1) { havePrev = false; continue; }
        int rr = (int)(fr * R);
        int px = cx + (int)(t.sinB[j] * rr);
        int py = cy - (int)(t.cosB[j] * rr);
        if (havePrev) gfx->drawLine(prevPx, prevPy, px, py, trailColor);
        prevPx = px; prevPy = py; havePrev = true;
      }
  }
  // Key sits in the corner of the radar area, clear of the circle.
  if (slotTo >= TRAIL_SLOTS && cfg.sKey)
    drawTrailKey(L.full ? 830 : 870, L.full ? 420 : 460,
                 cfg.cTrDep, cfg.cTrArr, cfg.cTrOver, cfg.cAirpt);
}

// Blips (projected positions) + their collision-avoided labels, with the
// traffic filter/watchlist applied: hide/only remove blips, highlight/alert
// recolor them (+ label + banner), quiet mode dims everything when nothing
// watched is airborne.
void drawRadarBlips(const RadarLayout &L, const RadarCfg &cfg) {
  const int cx = L.cx, cy = L.cy, R = L.R;
  float sweepDeg = fmodf(millis() / (cfg.swpSec * 1000.0f / 360.0f), 360.0f);
  float elapsed = (millis() - lastDataMs) / 1000.0f;

  // Filter pass 1: match every blip once; frame-level state.
  int ruleOf[MAX_BLIPS];
  bool hasOnly = false, anyWatchAirborne = false;
  for (uint8_t i = 0; i < blipCount; i++) {
    ruleOf[i] = filterMatchBlip(blips[i], cfg.rules);
    if (ruleOf[i] >= 0) {
      uint8_t a = cfg.rules[ruleOf[i]].action;
      if (a == FA_ONLY) hasOnly = true;
      if (a == FA_HIGHLIGHT || a == FA_ALERT) anyWatchAirborne = true;
    }
  }

  gfx->setTextSize(2);

  LabelRect placed[MAX_BLIPS];
  int placedCount = 0;
  const char *alertCs = nullptr;
  uint16_t alertColor = 0;

  uint8_t maxB = cfg.maxBlips < blipCount ? cfg.maxBlips : blipCount;
  for (uint8_t i = 0; i < maxB; i++) {
      int rule = ruleOf[i];
      if (rule >= 0 && cfg.rules[rule].action == FA_HIDE) continue;
      if (hasOnly && (rule < 0 || cfg.rules[rule].action != FA_ONLY)) continue;

      double la = blips[i].lat, lo = blips[i].lon;
      if (blips[i].speedMs > 0 && elapsed > 0)
        projectLatLon(blips[i].lat, blips[i].lon, blips[i].track, blips[i].speedMs * elapsed, la, lo);

      double dist = haversineKm(cfg.hLat, cfg.hLon, la, lo);
      float fr = (float)(dist / cfg.rMax);
      if (fr > 1) continue;

      int rr = (int)(fr * R);
      double brg = bearingDeg(cfg.hLat, cfg.hLon, la, lo);
      int bx = cx + (int)(sin(deg2rad(brg)) * rr);
      int by = cy - (int)(cos(deg2rad(brg)) * rr);

      float behind = fmodf(sweepDeg - (float)brg + 360.0f, 360.0f);
      uint16_t blipColor, labelColor;
      if (rule >= 0 && (cfg.rules[rule].action == FA_HIGHLIGHT || cfg.rules[rule].action == FA_ALERT)) {
        blipColor = labelColor = cfg.rules[rule].color;
        if (cfg.rules[rule].action == FA_ALERT && !alertCs && blips[i].callsign[0]) {
          alertCs = blips[i].callsign;
          alertColor = cfg.rules[rule].color;
        }
      } else if (cfg.quiet && !anyWatchAirborne) {
        blipColor = labelColor = DARKGREY;
      } else {
        blipColor = (behind < 30) ? cfg.cBlipHi : cfg.cBlip;
        labelColor = WHITE;
      }
      if (strcmp(blips[i].category, "A7") == 0)
        drawHelicopterIcon(bx, by, blips[i].track, blipColor);
      else
        drawPlaneIcon(bx, by, blips[i].track, blipColor);

      {
        char lines[9][12];
        int lineCount = 0;
        bool hasCallsign = blips[i].callsign[0] && strcmp(blips[i].callsign, "(no id)") != 0;

        if (cfg.showCS && hasCallsign) {
          strncpy(lines[lineCount], blips[i].callsign, 11); lines[lineCount][11] = '\0'; lineCount++;
        }
        if (cfg.showFlt) {
          float altFt = blips[i].altitudeM * 3.28084f;
          snprintf(lines[lineCount], 12, "FL%03.0f", altFt / 100.0f); lineCount++;
        }
        if (cfg.showRte && blips[i].hasRoute) {
          snprintf(lines[lineCount], 12, "%s>%s", blips[i].dep, blips[i].arr); lineCount++;
        }
        if (cfg.showAir) {
          const char* airline = blips[i].airline[0] ? blips[i].airline : lookupAirline(blips[i].callsign);
          if (airline) {
            strncpy(lines[lineCount], airline, 10); lines[lineCount][10] = '\0'; lineCount++;
          }
        }
        if (cfg.showRg && blips[i].reg[0]) {
          strncpy(lines[lineCount], blips[i].reg, 11); lines[lineCount][11] = '\0'; lineCount++;
        }
        if (cfg.showSq && blips[i].squawk[0]) {
          snprintf(lines[lineCount], 12, "SQ%s", blips[i].squawk); lineCount++;
        }
        if (cfg.showVr) {
          snprintf(lines[lineCount], 12, "%+dfpm", blips[i].vrateFpm); lineCount++;
        }
        if (cfg.showTy && blips[i].typeCode[0]) {
          strncpy(lines[lineCount], blips[i].typeCode, 11); lines[lineCount][11] = '\0'; lineCount++;
        }
        if (cfg.showSpd) {
          float speedKt = blips[i].speedMs * 1.94384f;
          snprintf(lines[lineCount], 12, "%ukn", (unsigned int)(speedKt + 0.5f)); lineCount++;
        }

        placeLabel(placed, placedCount, MAX_BLIPS, bx, by, lines, lineCount,
                   L.boundX, L.boundY, L.boundW, L.boundH, labelColor);
      }
    }

  // Alert banner: a watched aircraft is in range. Top-center of the radar
  // area, filled strip in the rule's color.
  if (alertCs) {
    char msg[24];
    snprintf(msg, sizeof(msg), "ALERT: %s", alertCs);
    gfx->setTextSize(3);
    int16_t x1, y1; uint16_t tw, th;
    gfx->getTextBounds(msg, 0, 0, &x1, &y1, &tw, &th);
    int bx = cx - (int)tw / 2;
    int by = L.full ? 45 : L.boundY + 10;
    gfx->fillRect(bx - 8, by - 4, tw + 16, th + 10, alertColor);
    gfx->setTextColor(BLACK, alertColor);
    gfx->setCursor(bx, by);
    gfx->print(msg);
  }
}

// Renders a QR code (Nayuki qrcodegen, vendored) with a quiet-zone border.
// Used for the config-page URL on the Weather & System screen.
static void drawQrCode(int x, int y, int scale, const char *text, uint16_t fg, uint16_t bg) {
  static uint8_t qrcode[qrcodegen_BUFFER_LEN_MAX];
  static uint8_t temp[qrcodegen_BUFFER_LEN_MAX];
  if (!qrcodegen_encodeText(text, temp, qrcode, qrcodegen_Ecc_LOW,
                            qrcodegen_VERSION_MIN, qrcodegen_VERSION_MAX,
                            qrcodegen_Mask_AUTO, true)) return;
  int size = qrcodegen_getSize(qrcode);
  gfx->fillRect(x - 4 * scale, y - 4 * scale, (size + 8) * scale, (size + 8) * scale, bg);
  for (int qy = 0; qy < size; qy++)
    for (int qx = 0; qx < size; qx++)
      if (qrcodegen_getModule(qrcode, qx, qy))
        gfx->fillRect(x + qx * scale, y + qy * scale, scale, scale, fg);
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

  // Network identity + setup-AP status (WiFi management feature)
  gfx->setCursor(500, 340);
  if (wifiApFallbackActive) {
    gfx->setTextColor(YELLOW, BLACK);
    gfx->print("WiFi: SETUP AP 'PlaneSpotter-Setup'   ");
    gfx->setCursor(500, 380);
    gfx->print("Join it, open 192.168.4.1   ");
    gfx->setTextColor(WHITE, BLACK);
  } else if (WiFi.status() == WL_CONNECTED) {
    gfx->printf("WiFi: %-16.16s   ", WiFi.SSID().c_str());
    gfx->setCursor(500, 380);
    gfx->printf("IP: %s   ", WiFi.localIP().toString().c_str());
  } else {
    gfx->print("WiFi: reconnecting...   ");
  }

  gfx->setCursor(60, 360);
  gfx->printf("RAM Free: %d KB   ", ESP.getFreeHeap() / 1024);

  gfx->setCursor(500, 360);
  gfx->printf("PSRAM Free: %d KB   ", ESP.getFreePsram() / 1024);

  gfx->setCursor(60, 420);
  gfx->printf("API Req: %lu OK / %lu FAIL   ", stats.requestsOk, stats.requestsFail);

  // Config-page URL as text + QR code — point a phone camera at it to open
  // the web UI without typing an IP. In setup-AP mode the QR points at the
  // AP's config address instead.
  {
    char url[40];
    if (wifiApFallbackActive) strcpy(url, "http://192.168.4.1/");
    else snprintf(url, sizeof(url), "http://%s/", WiFi.localIP().toString().c_str());
    gfx->setTextSize(2);
    gfx->setTextColor(CYAN, BLACK);
    gfx->setCursor(60, 480);
    gfx->printf("Config: %s   ", url);
    drawQrCode(830, 430, 4, url, BLACK, WHITE);
  }
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

  // Per-frame timing breakdown (see render()) -- for diagnosing the
  // "stepped, not smooth" sweep animation without needing USB serial.
  server.on("/timingdebug", []() {
    char buf[256];
    snprintf(buf, sizeof(buf),
      "fillScreen: %lu us\nscreen draw: %lu us\npushFrame:  %lu us\ntotal work: %lu us\nactual frame interval: %lu ms\n",
      (unsigned long)lastFillUs, (unsigned long)lastDrawUs, (unsigned long)lastPushUs,
      (unsigned long)(lastFillUs + lastDrawUs + lastPushUs), (unsigned long)lastFrameIntervalMs);
    server.send(200, "text/plain", buf);
  });

  // Device health snapshot for remote monitoring (slowdown/crash-watch):
  // memory floors, reset reason, frame timings, data-fetch stats.
  server.on("/health", []() {
    char buf[768];
    snprintf(buf, sizeof(buf),
      "uptime_s: %lu\nreset_reason: %d\nheap_free: %u\nheap_min_free: %u\nheap_largest_block: %u\npsram_free: %u\n"
      "rssi_dbm: %d\nscreen: %u\nfill_us: %lu\ndraw_us: %lu\npush_us: %lu\nframe_interval_ms: %lu\n"
      "blips: %u\napi_ok: %lu\napi_fail: %lu\nfetch_stage: %u\nfetch_http: %d\nfetch_bytes: %lu\nfetch_head: %s\n"
      "top5: %u\nnearest_valid: %d\nnearest_cs: %s\nrange_km: %.0f\ndata_age_ms: %lu\nstaged_slice: %d\nlast_cycle_age_ms: %lu\ndraw_buf: %u\nglow0: %u\n"
      "touch_xy: %d,%d\ntouch_action: %s\n",
      (unsigned long)(millis() / 1000), (int)esp_reset_reason(),
      (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(), (unsigned)ESP.getMaxAllocHeap(), (unsigned)ESP.getFreePsram(),
      (int)WiFi.RSSI(), screen,
      (unsigned long)lastFillUs, (unsigned long)lastDrawUs, (unsigned long)lastPushUs,
      (unsigned long)lastFrameIntervalMs,
      blipCount, (unsigned long)stats.requestsOk, (unsigned long)stats.requestsFail,
      (unsigned)stats.lastFetchStage, stats.lastHttpCode, (unsigned long)stats.lastPayloadBytes,
      stats.lastPayloadHead,
      top5Count, (int)nearest.valid, nearest.callsign,
      radarMaxKm, (unsigned long)(millis() - lastDataMs),
      (int)slice, (unsigned long)(millis() - lastFullCycle), (unsigned)drawBufIdx, (unsigned)glowCount[0],
      (int)dbgTouchX, (int)dbgTouchY, dbgTouchAction);
    server.send(200, "text/plain", buf);
  });

  initRGBPanel();
  directGfx->setBuffer(panelFbs[drawBufIdx]);  // inject BEFORE begin() so begin() skips its own allocation
  if (!directGfx->begin()) {
    Serial.println("[lcd7b] Failed to initialize display!");
  }
  // Staging canvases for the sliced radar redraw (2x own 1.2MB PSRAM
  // buffers, ping-pong). bgFrame starts as an all-black fallback; the first
  // completed staged cycle (or screen-entry sync) makes it a valid source.
  stagingCanvas[0] = new Arduino_Canvas(LCD_W, LCD_H, nullptr);
  stagingCanvas[1] = new Arduino_Canvas(LCD_W, LCD_H, nullptr);
  if (!stagingCanvas[0]->begin() || !stagingCanvas[1]->begin()) {
    Serial.println("[lcd7b] Failed to allocate staging canvases!");
  }
  stagingCanvas[0]->fillScreen(BLACK);
  stagingCanvas[1]->fillScreen(BLACK);
  bgFrame = stagingCanvas[0];
  gfx->fillScreen(BLACK);
  gfx->setCursor(20, 100);
  gfx->setTextSize(4);
  gfx->setTextColor(GREEN);
  gfx->print("CYD PLANE SPOTTER (zero-copy dbuf)");
  pushFrame();
  delay(1500);
}

void connectWiFiShow() {
  gfx->fillScreen(BLACK);
  gfx->setCursor(20, 100);
  gfx->setTextSize(3);
  gfx->setTextColor(WHITE);
  gfx->print("Connecting WiFi...");
  pushFrame();
  connectWiFi();
  if (wifiApFallbackActive) {
    // No saved network reachable — leave setup instructions on screen for a
    // while so the user can fix credentials without USB/serial.
    memset(panelFbs[drawBufIdx], 0, (size_t)LCD_W * LCD_H * sizeof(uint16_t));
    gfx->setTextSize(4);
    gfx->setTextColor(YELLOW, BLACK);
    gfx->setCursor(20, 80);
    gfx->print("WIFI SETUP MODE");
    gfx->setTextSize(3);
    gfx->setTextColor(WHITE, BLACK);
    gfx->setCursor(20, 160);
    gfx->print("1. Connect to WiFi: PlaneSpotter-Setup");
    gfx->setCursor(20, 210);
    gfx->print("2. Open http://192.168.4.1 in a browser");
    gfx->setCursor(20, 260);
    gfx->print("3. Enter your WiFi details and save");
    gfx->setTextColor(CYAN, BLACK);
    gfx->setCursor(20, 330);
    gfx->print("The radar will join your network automatically.");
    pushFrame();
    delay(8000);
  }
  gfx->fillScreen(BLACK);
  pushFrame();
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

// --- Staged (scheduled) radar redraw -------------------------------------
// A monolithic full redraw costs ~200-300ms (fill + trails/blips + present
// + back-buffer sync) and used to run as one blocking burst every
// radarRedrawMs — the sweep visibly froze twice a second. Instead, the next
// frame is rendered into a third PSRAM buffer (stagingCanvas, ping-pong) in
// small slices in the gaps between sweep ticks, band-copied into the
// inactive panel fb, swapped in with ONE atomic present, then mirrored into
// the other fb in 4 more band slices.
//
// Sweep update modes, chosen by phase (this is what fixed the "jittery
// sweep" report):
//  - IDLE and RENDER phases (fill/static/trails/blips): the slices only
//    touch the staging canvas, both panel fbs stay in sync, so the sweep
//    uses the GOOD zero-copy path (erase+draw in the INACTIVE fb, then an
//    O(1) present) — never visible mid-scan.
//  - COPY/SWAP/SYNC phase: the two fbs temporarily differ, so presents
//    would alternate content. The sweep is then drawn directly into the
//    ACTIVE fb, but ONLY right after VSYNC (vblank, ~4ms window) — an
//    arbitrary-timing active-fb erase+rewrite splits the line at the scan
//    position, which was exactly the reported jitter.
//
// Sweep erase position history is tracked PER FRAMEBUFFER (glowX/glowY/
// glowCount, declared with the panel state near the top of the file): the
// two driver buffers alternate on every present, so each buffer must
// remember where ITS OWN line(s) were, or the erase misses and ghosts.
// Erases RESTORE background from bgFrame (the last completed staged frame,
// sweep-free) — erasing to BLACK cut gashes through labels/trails.
//
// Phosphor afterglow: instead of a single previous position per fb, each
// tick restores the whole previous endpoint set from bgFrame and redraws it
// dimmest-first behind the new line — a CRT-style fading trail. History
// length 1 degrades to the plain line.
static uint16_t dim565(uint16_t c, uint8_t num, uint8_t den) {
  uint16_t r = ((c >> 11) & 0x1F) * num / den;
  uint16_t g = ((c >> 5) & 0x3F) * num / den;
  uint16_t b = (c & 0x1F) * num / den;
  return (r << 11) | (g << 5) | b;
}
// Brightness per segment age (newest first), tenths — roughly exponential.
static const uint8_t GLOW_FADE[GLOW_MAX] = { 10, 7, 5, 4, 3, 2, 2, 1, 1, 1, 1, 1 };

// One sweep update for one framebuffer. Restores last tick's glow from the
// background frame, shifts the new endpoint in, redraws dimmest-first so
// the heavy center overlap ends up brightest.
static void sweepGlowTick(uint8_t idx, int cx, int cy, int newX, int newY, uint16_t col, uint8_t K) {
  uint16_t *fb = panelFbs[idx];
  const uint16_t *bg = bgFrame->getFramebuffer();
  for (uint8_t i = 0; i < glowCount[idx]; i++)
    restoreLine(fb, bg, cx, cy, glowX[idx][i], glowY[idx][i]);
  if (K > GLOW_MAX) K = GLOW_MAX;
  if (K == 0) K = 1;
  uint8_t n = glowCount[idx] < K ? glowCount[idx] : (uint8_t)(K - 1);
  for (int i = n; i > 0; i--) { glowX[idx][i] = glowX[idx][i-1]; glowY[idx][i] = glowY[idx][i-1]; }
  glowX[idx][0] = newX; glowY[idx][0] = newY;
  if (glowCount[idx] < K) glowCount[idx]++;
  for (int i = glowCount[idx] - 1; i >= 0; i--)
    plotLine(fb, cx, cy, glowX[idx][i], glowY[idx][i], dim565(col, GLOW_FADE[i], 10));
}
static void sweepGlowSeed(uint8_t idx, int x, int y) {
  glowX[idx][0] = x; glowY[idx][0] = y; glowCount[idx] = 1;
}

// Executes exactly one slice of the staged redraw and advances the state
// machine. Called only when no sweep tick is due, so the sweep never waits
// on content rendering.
void runStagingSlice(const RadarLayout &L) {
  const int bandH = LCD_H / 8;  // copy/sync band height (8 slices per fb copy)
  const size_t bandBytes = (size_t)bandH * LCD_W * sizeof(uint16_t);
  Arduino_Canvas *stg = stagingCanvas[stgRenderIdx];
  switch (slice) {
    case SL_FILL_A: case SL_FILL_B: case SL_FILL_C: case SL_FILL_D: {
      // Raw memset in quarters (~10ms each) — smaller slices keep the sweep
      // tick cadence (and with it the afterglow spacing) regular. ROM memset
      // does 32-bit stores in a single flat call.
      int band = slice - SL_FILL_A;
      const size_t q = (size_t)LCD_W * LCD_H / 4 * sizeof(uint16_t);
      memset((uint8_t *)stg->getFramebuffer() + (size_t)band * q, 0, q);
      slice = (band == 3) ? SL_STATIC : (StageSlice)(slice + 1);
      break;
    }
    case SL_STATIC: {
      RadarCfg cfg = radarCfgSnapshot();
      if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
      drawRadarFurniture(L, cfg);
      drawRadarStatic(L, cfg);  // samples trails, records drawnSweepX/Y
      if (dataMutex) xSemaphoreGive(dataMutex);
      slice = SL_TRAILS_A;
      break;
    }
    case SL_TRAILS_A: case SL_TRAILS_B: case SL_TRAILS_C: case SL_TRAILS_D: {
      RadarCfg cfg = radarCfgSnapshot();
      uint8_t from = (uint8_t)((slice - SL_TRAILS_A) * 5);
      uint8_t to = from + 5;
      if (to > TRAIL_SLOTS) to = TRAIL_SLOTS;
      if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
      drawRadarTrails(L, cfg, from, to);
      if (dataMutex) xSemaphoreGive(dataMutex);
      slice = (slice == SL_TRAILS_D) ? SL_BLIPS : (StageSlice)(slice + 1);
      break;
    }
    case SL_BLIPS: {
      RadarCfg cfg = radarCfgSnapshot();
      if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
      drawRadarBlips(L, cfg);
      if (dataMutex) xSemaphoreGive(dataMutex);
      gfx = directGfx;
      slice = SL_COPY_A;
      break;
    }
    case SL_COPY_A: case SL_COPY_B: case SL_COPY_C: case SL_COPY_D:
    case SL_COPY_E: case SL_COPY_F: case SL_COPY_G: case SL_COPY_H: {
      // staging -> inactive fb, one band per slice. drawBufIdx's fb is free:
      // the last present's waitFrameDone() guaranteed it, and nothing
      // presents during the copy phase.
      int band = slice - SL_COPY_A;
      memcpy((uint8_t *)panelFbs[drawBufIdx] + (size_t)band * bandBytes,
             (uint8_t *)stg->getFramebuffer() + (size_t)band * bandBytes,
             bandBytes);
      slice = (band == 7) ? SL_SWAP : (StageSlice)(slice + 1);
      break;
    }
    case SL_SWAP:
      // The ONE atomic content swap of the cycle. The staged frame is now
      // displayed, so it also becomes the valid background source for
      // sweep-line erases from here on.
      pushFrame();
      bgFrame = stagingCanvas[stgRenderIdx];
      sweepGlowSeed(drawBufIdx ^ 1, drawnSweepX, drawnSweepY);
      slice = SL_SYNC_A;
      break;
    case SL_SYNC_A: case SL_SYNC_B: case SL_SYNC_C: case SL_SYNC_D:
    case SL_SYNC_E: case SL_SYNC_F: case SL_SYNC_G: case SL_SYNC_H: {
      // Mirror the staged frame into the other fb (free since SL_SWAP's
      // waitFrameDone) so buffer flips don't alternate content.
      int band = slice - SL_SYNC_A;
      memcpy((uint8_t *)panelFbs[drawBufIdx] + (size_t)band * bandBytes,
             (uint8_t *)stg->getFramebuffer() + (size_t)band * bandBytes,
             bandBytes);
      if (band == 7) {
        sweepGlowSeed(drawBufIdx, drawnSweepX, drawnSweepY);
        slice = SL_IDLE;
        lastFullCycle = millis();
      } else {
        slice = (StageSlice)(slice + 1);
      }
      break;
    }
    default:
      slice = SL_IDLE;
      break;
  }
}

void renderRadar(bool justEntered) {
  static uint32_t lastSweep = 0;
  uint32_t now = millis();

  uint8_t cur = screen % LCD7B_NUM_SCREENS;
  const RadarLayout L = radarLayout(cur);

  // The fast path needs these config values without a full snapshot.
  float swpSec; uint16_t cSweepCol, redrawMs; uint8_t glowK; bool glowOn;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  swpSec = sweepPeriodSec; cSweepCol = colSweep; redrawMs = radarRedrawMs;
  glowOn = sweepGlow; glowK = sweepGlowLen;
  if (configMutex) xSemaphoreGive(configMutex);
  if (!glowOn) glowK = 1;

  // Screen switch: abandon any in-progress staging and do one immediate
  // monolithic redraw, so the new screen appears at once, fully correct.
  if (justEntered) {
    slice = SL_IDLE;
    gfx = directGfx;
    directGfx->setBuffer(panelFbs[drawBufIdx]);

    uint32_t t0 = micros();
    memset(panelFbs[drawBufIdx], 0, (size_t)LCD_W * LCD_H * sizeof(uint16_t));  // raw fill — see SL_FILL_A
    uint32_t t1 = micros();
    if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
    drawRadarAll(L);
    if (dataMutex) xSemaphoreGive(dataMutex);
    uint32_t t2 = micros();

    // Seed this buffer's erase position from the endpoint the screen
    // actually drew (recorded at draw time — NOT a fresh millis() here;
    // recomputing after the slow draw/push was the original ghosting bug).
    sweepGlowSeed(drawBufIdx, drawnSweepX, drawnSweepY);

    pushFrame();
    uint32_t t3 = micros();
    lastFillUs = t1 - t0; lastDrawUs = t2 - t1; lastPushUs = t3 - t2;

    // Sync the OTHER framebuffer to the frame just presented (see SL_SYNC).
    memcpy(panelFbs[drawBufIdx], panelFbs[drawBufIdx ^ 1], (size_t)LCD_W * LCD_H * sizeof(uint16_t));
    // Keep the background-source frame in step with what's on screen, so
    // sweep-line erases restore the right pixels.
    memcpy(bgFrame->getFramebuffer(), panelFbs[drawBufIdx ^ 1], (size_t)LCD_W * LCD_H * sizeof(uint16_t));
    sweepGlowSeed(drawBufIdx, drawnSweepX, drawnSweepY);

    lastFullCycle = now;
    lastSweep = now;
    return;
  }

  bool tickDue = (now - lastSweep >= 30);
  bool copyPhase = (slice >= SL_COPY_A);  // fbs differ here — no presents allowed

  if (tickDue) {
    float sweepDeg = fmodf(millis() / (swpSec * 1000.0f / 360.0f), 360.0f);
    double sw = deg2rad(sweepDeg);
    int newX = L.cx + (int)(sin(sw) * L.R);
    int newY = L.cy - (int)(cos(sw) * L.R);

    if (!copyPhase) {
      // Zero-copy tick (IDLE and render phases): afterglow update in the
      // inactive fb, then an O(1) present. Never visible mid-scan.
      sweepGlowTick(drawBufIdx, L.cx, L.cy, newX, newY, cSweepCol, glowK);
      pushFrame();
    } else {
      // Copy/swap/sync phase: present would alternate content, so draw
      // directly into the ACTIVE fb — but only right after VSYNC, so the
      // write lands in the vblank gap (~4ms). A full afterglow update (up to
      // 24 line writes) would overflow vblank into the active scan and
      // reintroduce splits, so this phase uses the plain single line.
      uint8_t act = drawBufIdx ^ 1;
      uint32_t v0 = vsyncCount, tW = millis();
      while (vsyncCount == v0 && millis() - tW < 60) { delay(1); }
      sweepGlowTick(act, L.cx, L.cy, newX, newY, cSweepCol, 1);
    }
    // Timestamp AFTER the blocking present — not at tick start. The tick's
    // own frame-completion wait (~46ms) used to count toward the 30ms
    // throttle, so tickDue was true on every call and staging slices
    // starved (~20s per content cycle — the "planes frozen / zoom dead"
    // report). Setting it here frees most of each frame period for slices.
    lastSweep = millis();
    return;
  }

  if (slice != SL_IDLE) {
    runStagingSlice(L);
    return;
  }

  // Idle between cycles and no tick due: kick off a new staged cycle.
  if (now - lastFullCycle >= redrawMs) {
    stgRenderIdx ^= 1;  // ping-pong: render into the OTHER staging buffer, keeping the last complete frame valid as bgFrame
    gfx = stagingCanvas[stgRenderIdx];
    slice = SL_FILL_A;
  }
}

void render() {
  // Tracked locally rather than via the shared `lastScreen` global: that
  // global only gets touched from inside whichever path actually redraws,
  // and since the radar path no longer runs on every render() call the way
  // the others do, comparing against it here would go stale across quick
  // navigation between screens. This little bit of state is dedicated to
  // "did we just switch screens," independent of either path's own cadence.
  static uint8_t prevScreenIdx = 255;  // sentinel forces a full redraw on first-ever call
  uint8_t curScreenIdx = screen % LCD7B_NUM_SCREENS;
  bool justEntered = (curScreenIdx != prevScreenIdx);
  prevScreenIdx = curScreenIdx;

  if (curScreenIdx == 2 || curScreenIdx == 4) {
    renderRadar(justEntered);
    return;
  }

  // Abandon any staged radar redraw interrupted by the screen switch, and
  // restore the global render target to a panel fb (staging slices retarget
  // gfx to the staging canvas).
  slice = SL_IDLE;
  gfx = directGfx;
  directGfx->setBuffer(panelFbs[drawBufIdx]);

  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (!justEntered && now - lastDraw < 100) return;  // no animation on these screens, no need for a tight cadence
  uint32_t frameInterval = now - lastDraw;
  lastDraw = now;

  uint32_t t0 = micros();
  memset(panelFbs[drawBufIdx], 0, (size_t)LCD_W * LCD_H * sizeof(uint16_t));  // raw fill — see SL_FILL_A
  uint32_t t1 = micros();

  if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
  switch (screen % LCD7B_NUM_SCREENS) {
    case 0: screenTargetIntel();   break;
    case 1: screenTop5();          break;
    case 3: screenWeatherSystem(); break;
  }
  if (dataMutex) xSemaphoreGive(dataMutex);
  uint32_t t2 = micros();

  pushFrame();
  uint32_t t3 = micros();

  lastFillUs = t1 - t0;
  lastDrawUs = t2 - t1;
  lastPushUs = t3 - t2;
  lastFrameIntervalMs = frameInterval;
}

void checkTouch() {
  if (!touch.touched()) return;

  uint16_t x, y;
  touch.readData(&x, &y);
  dbgTouchX = x; dbgTouchY = y;

  if (millis() - lastTouchMs < 400) return;  // debounce, matches the other boards
  lastTouchMs = millis();

  int sx = constrain((int)x, 0, LCD_W - 1);
  int sy = constrain((int)y, 0, LCD_H - 1);

  uint8_t cur = screen % LCD7B_NUM_SCREENS;
  if (cur == 2 || cur == 4) {
    // Radar screens: check +/- range buttons (classic layout keeps them in
    // the left info column; the full-screen layout has its own bigger set
    // in the bottom-left margin).
    const int bx  = (cur == 4) ? FRNG_BTN_X   : RNG_BTN_X;
    const int bw  = (cur == 4) ? FRNG_BTN_W   : RNG_BTN_W;
    const int bh  = (cur == 4) ? FRNG_BTN_H   : RNG_BTN_H;
    const int py  = (cur == 4) ? FRNG_PLUS_Y  : RNG_PLUS_Y;
    const int my  = (cur == 4) ? FRNG_MINUS_Y : RNG_MINUS_Y;
    if (sx >= bx && sx <= bx + bw && sy >= py && sy <= py + bh) {
      if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
      radarMaxKm = min(200.0f, radarMaxKm + 10.0f);
      if (configMutex) xSemaphoreGive(configMutex);
      markRangeDirty();
      snprintf(dbgTouchAction, sizeof(dbgTouchAction), "zoom+ rng=%d", (int)radarMaxKm);
      Serial.printf("[lcd7b] Range increased: %d km\n", (int)radarMaxKm);
      return;
    }
    if (sx >= bx && sx <= bx + bw && sy >= my && sy <= my + bh) {
      if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
      radarMaxKm = max(10.0f, radarMaxKm - 10.0f);
      if (configMutex) xSemaphoreGive(configMutex);
      markRangeDirty();
      snprintf(dbgTouchAction, sizeof(dbgTouchAction), "zoom- rng=%d", (int)radarMaxKm);
      Serial.printf("[lcd7b] Range decreased: %d km\n", (int)radarMaxKm);
      return;
    }
  }

  // Default: cycle to next screen
  snprintf(dbgTouchAction, sizeof(dbgTouchAction), "cycle scr=%d", (screen + 1) % LCD7B_NUM_SCREENS);
  screen = (screen + 1) % LCD7B_NUM_SCREENS;
}
