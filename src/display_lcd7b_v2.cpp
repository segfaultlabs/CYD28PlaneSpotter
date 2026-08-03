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
// signal available on every blip.
enum TrailClass : uint8_t { TC_OVER = 0, TC_DEP, TC_ARR };
#define CLASS_NEAR_AIRPORT_KM 25.0
#define CLASS_MAX_ALT_FT 12000.0f
#define CLASS_VRATE_FPM 250
static const uint16_t TRAIL_COLORS[3] = { YELLOW, CYAN, ORANGE };  // TC_OVER, TC_DEP, TC_ARR
static TrailClass classifyBlip(const Blip &b) {
  if (b.altitudeM * 3.28084f >= CLASS_MAX_ALT_FT) return TC_OVER;
  for (uint8_t a = 0; a < AIRPORT_COUNT; a++) {
    if (haversineKm(b.lat, b.lon, AIRPORTS[a].lat, AIRPORTS[a].lon) <= CLASS_NEAR_AIRPORT_KM) {
      if (b.vrateFpm > CLASS_VRATE_FPM) return TC_DEP;
      if (b.vrateFpm < -CLASS_VRATE_FPM) return TC_ARR;
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
  // ~22MHz tested ceiling for octal PSRAM @80MHz, no margin). Settled at
  // 20MHz (~21.8Hz physical refresh). Going beyond needs the 120MHz-PSRAM
  // custom-sdkconfig rebuild — see LCD7B_V2_DEBUG_LOG.md.
  cfg.timings.pclk_hz = 20000000;
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
DirectCanvas *gfx = new DirectCanvas(LCD_W, LCD_H);

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
  gfx->setBuffer(panelFbs[drawBufIdx]);
}

IOExtension ioExpander;
GT911_Touch touch(TOUCH_INT, ioExpander);

#define LCD7B_NUM_SCREENS 5  // Target Intel, Top 5, Radar, Weather & System, Radar Full

// Per-frame timing breakdown, updated every render() call, read via
// GET /timingdebug -- so we can see exactly where a frame's time goes
// (canvas clear vs. screen draw vs. driver push) instead of guessing.
static volatile uint32_t lastFillUs = 0, lastDrawUs = 0, lastPushUs = 0, lastFrameIntervalMs = 0;

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

// Diamond marker for airports — distinct shape AND color (MAGENTA) so it
// can't be confused with blips, traces, rings, or the sweep.
static void drawAirportMarker(int px, int py, uint16_t color) {
  gfx->drawLine(px, py - 6, px + 6, py, color);
  gfx->drawLine(px + 6, py, px, py + 6, color);
  gfx->drawLine(px, py + 6, px - 6, py, color);
  gfx->drawLine(px - 6, py, px, py - 6, color);
}

// On-screen key explaining the trace classification colors + airport
// marker. Drawn only when showTraces is on (no traces, nothing to explain).
static void drawTrailKey(int x, int y) {
  const char *labels[4] = { "TAKEOFF", "LANDING", "FLYOVER", "AIRPORT" };
  gfx->setTextSize(2);
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t c = (i < 3) ? TRAIL_COLORS[i] : MAGENTA;
    if (i == 3) drawAirportMarker(x + 10, y + 8, MAGENTA);
    else gfx->drawFastHLine(x, y + 8, 20, c);
    gfx->setTextColor(c, BLACK);
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
};
static RadarCfg radarCfgSnapshot() {
  RadarCfg c;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  c.hLat = homeLat; c.hLon = homeLon; c.rMax = radarMaxKm;
  c.showCS = showCallsign; c.showAir = showAirline; c.showSpd = showSpeed; c.showFlt = showFL; c.showRte = showRoute;
  c.showRg = showReg; c.showSq = showSquawk; c.showVr = showVRate; c.showTy = showType;
  if (configMutex) xSemaphoreGive(configMutex);
  return c;
}
void drawRadarCommon(const RadarLayout &L);  // defined below screenRadar()

void screenRadar() {
  drawHeader("RADAR");

  RadarCfg cfg = radarCfgSnapshot();

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
  gfx->printf("RNG %d km   ", (int)cfg.rMax);

  gfx->fillRoundRect(RNG_BTN_X, RNG_PLUS_Y, RNG_BTN_W, RNG_BTN_H, 6, DARKGREY);
  gfx->setTextColor(WHITE, DARKGREY);
  gfx->setCursor(RNG_BTN_X + 75, RNG_PLUS_Y + 18);
  gfx->print("+");

  gfx->fillRoundRect(RNG_BTN_X, RNG_MINUS_Y, RNG_BTN_W, RNG_BTN_H, 6, DARKGREY);
  gfx->setCursor(RNG_BTN_X + 75, RNG_MINUS_Y + 18);
  gfx->print("-");

  drawRadarCommon(radarLayout(2));
}

// Everything in/around the radar circle, shared by both radar screens:
// rings/axes/compass, airport markers, sweep line, traces (+ color key),
// blips, labels. Reads its own config snapshot. Full redraws only — the
// sweep's fast per-frame update lives in renderRadar().
void drawRadarCommon(const RadarLayout &L) {
  RadarCfg cfg = radarCfgSnapshot();
  const int cx = L.cx, cy = L.cy, R = L.R;
  uint32_t nowMs = millis();
  float sweepDeg = fmodf(millis() / 15.0f, 360.0f);
  double sw = deg2rad(sweepDeg);

  gfx->drawCircle(cx, cy, R, DARKGREY);
  gfx->drawCircle(cx, cy, R * 2 / 3, DARKGREY);
  gfx->drawCircle(cx, cy, R / 3, DARKGREY);
  gfx->drawLine(cx - R, cy, cx + R, cy, DARKGREY);
  gfx->drawLine(cx, cy - R, cx, cy + R, DARKGREY);

  // Compass — matches the same north-up, clockwise convention already
  // used for blip placement (bx = cx + sin(brg)*r, by = cy - cos(brg)*r,
  // brg 0=N/90=E/180=S/270=W), so these letters are correct relative to
  // where blips actually land, not just decorative. Classic layout puts
  // them OUTSIDE the circle (header/column leave margin); the full-screen
  // layout's circle nearly touches the canvas edges, so they go INSIDE.
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

  // --- Physical airports in range: MAGENTA diamond + IATA code ---
  gfx->setTextColor(MAGENTA, BLACK);
  for (uint8_t a = 0; a < AIRPORT_COUNT; a++) {
    double dist = haversineKm(cfg.hLat, cfg.hLon, AIRPORTS[a].lat, AIRPORTS[a].lon);
    float fr = (float)(dist / cfg.rMax);
    if (fr > 1) continue;
    int rr = (int)(fr * R);
    double brg = bearingDeg(cfg.hLat, cfg.hLon, AIRPORTS[a].lat, AIRPORTS[a].lon);
    int px = cx + (int)(sin(deg2rad(brg)) * rr);
    int py = cy - (int)(cos(deg2rad(brg)) * rr);
    drawAirportMarker(px, py, MAGENTA);
    gfx->setCursor(px + 9, py - 8);
    gfx->print(AIRPORTS[a].iata);
  }

  drawnSweepX = cx + (int)(sin(sw) * R);
  drawnSweepY = cy - (int)(cos(sw) * R);
  gfx->drawLine(cx, cy, drawnSweepX, drawnSweepY, GREEN);

  float elapsed = (millis() - lastDataMs) / 1000.0f;

  // --- Flight path traces: per-aircraft position history, keyed by
  // callsign since blips[] array indices aren't stable between fetch
  // cycles. Sampled once per render cycle. Entries not seen in
  // TRAIL_STALE_MS are dropped, so a trail vanishes once its aircraft is
  // no longer tracked (out of range / off screen). Color encodes flight
  // phase (see classifyBlip): CYAN takeoff / ORANGE landing near an
  // airport, YELLOW flyover — key drawn below.
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
        // "a path" the way a real radar trace does.
        const Blip *owner = findBlipByCallsign(t.callsign);
        uint16_t trailColor = owner ? TRAIL_COLORS[classifyBlip(*owner)] : TRAIL_COLORS[TC_OVER];
        int prevPx = 0, prevPy = 0; bool havePrev = false;
        for (uint16_t j = 0; j < t.count; j++) {
          double dist = haversineKm(cfg.hLat, cfg.hLon, t.lat[j], t.lon[j]);
          float fr = (float)(dist / cfg.rMax);
          if (fr > 1) { havePrev = false; continue; }
          int rr = (int)(fr * R);
          double brg = bearingDeg(cfg.hLat, cfg.hLon, t.lat[j], t.lon[j]);
          int px = cx + (int)(sin(deg2rad(brg)) * rr);
          int py = cy - (int)(cos(deg2rad(brg)) * rr);
          if (havePrev) gfx->drawLine(prevPx, prevPy, px, py, trailColor);
          prevPx = px; prevPy = py; havePrev = true;
        }
    }
    // Key sits in the corner of the radar area, clear of the circle.
    drawTrailKey(L.full ? 830 : 870, L.full ? 420 : 460);
  }

  gfx->setTextSize(2);

  LabelRect placed[MAX_BLIPS];
  int placedCount = 0;

  for (uint8_t i = 0; i < blipCount; i++) {
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
      uint16_t blipColor = (behind < 30) ? YELLOW : GREEN;
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
                   L.boundX, L.boundY, L.boundW, L.boundH);
      }
    }
}

// Full-screen radar (screen index 4): no header, no left column — the
// circle fills the whole canvas and everything else lives in the
// corners/margins. Zoom buttons are much bigger than the classic layout's
// column-cramped ones, per user request.
static const int FRNG_BTN_X = 20, FRNG_BTN_W = 195, FRNG_BTN_H = 85;
static const int FRNG_PLUS_Y = 405, FRNG_MINUS_Y = 500;

void screenRadarFull() {
  RadarCfg cfg = radarCfgSnapshot();
  drawRadarCommon(radarLayout(4));

  // Weather, top-left corner
  gfx->setTextSize(2);
  if (weather.valid) {
    drawWeatherIcon(15, 12, weather.code);
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
    char buf[512];
    snprintf(buf, sizeof(buf),
      "uptime_s: %lu\nreset_reason: %d\nheap_free: %u\nheap_min_free: %u\npsram_free: %u\n"
      "rssi_dbm: %d\nscreen: %u\nfill_us: %lu\ndraw_us: %lu\npush_us: %lu\nframe_interval_ms: %lu\n"
      "blips: %u\napi_ok: %lu\napi_fail: %lu\n",
      (unsigned long)(millis() / 1000), (int)esp_reset_reason(),
      (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(), (unsigned)ESP.getFreePsram(),
      (int)WiFi.RSSI(), screen,
      (unsigned long)lastFillUs, (unsigned long)lastDrawUs, (unsigned long)lastPushUs,
      (unsigned long)lastFrameIntervalMs,
      blipCount, (unsigned long)stats.requestsOk, (unsigned long)stats.requestsFail);
    server.send(200, "text/plain", buf);
  });

  initRGBPanel();
  gfx->setBuffer(panelFbs[drawBufIdx]);  // inject BEFORE begin() so begin() skips its own allocation
  if (!gfx->begin()) {
    Serial.println("[lcd7b] Failed to initialize display!");
  }
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

// Radar screen only: full clear+redraw of the whole canvas (rings/compass/
// blips/labels/left column) only periodically, when content can actually have
// changed; the sweep line gets its own much faster update in between — erase
// old position, draw new position, present. With zero-copy present, the
// fast tick needs NO dirty-region crop/copy at all (the old pushPartial()
// existed only to avoid a 1.2MB full-canvas copy per tick; a present is now
// an O(1) buffer-index flip, so the whole pushPartial/crop-buffer machinery
// is gone). The tick is gated by waitFrameDone() inside pushFrame(), so it
// naturally runs at the panel's physical refresh rate — the fastest the
// panel can ever show.
//
// Sweep erase position is tracked PER FRAMEBUFFER (prevSweepX/Y[2]): the two
// driver buffers alternate on every present, so the buffer a fast tick draws
// into contains the sweep line as of two ticks ago — each buffer must
// remember where ITS OWN line was, or the erase misses and ghosts.
void renderRadar(bool justEntered) {
  static uint32_t lastFull = 0;
  static uint32_t lastSweep = 0;
  static int prevSweepX[2] = { RADAR_CX, RADAR_CX };
  static int prevSweepY[2] = { RADAR_CY - RADAR_R, RADAR_CY - RADAR_R };
  static bool havePrevSweep[2] = { false, false };
  uint32_t now = millis();

  uint8_t cur = screen % LCD7B_NUM_SCREENS;
  const RadarLayout L = radarLayout(cur);

  bool needFull = justEntered || (now - lastFull >= 500);
  if (needFull) {
    lastFull = now;

    uint32_t t0 = micros();
    gfx->fillScreen(BLACK);
    uint32_t t1 = micros();
    if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
    if (cur == 4) screenRadarFull();
    else screenRadar();
    if (dataMutex) xSemaphoreGive(dataMutex);
    uint32_t t2 = micros();

    // Seed this buffer's erase position from the endpoint the screen
    // actually drew (recorded at draw time — NOT a fresh millis() here;
    // recomputing after the slow draw/push was the original ghosting bug).
    prevSweepX[drawBufIdx] = drawnSweepX;
    prevSweepY[drawBufIdx] = drawnSweepY;
    havePrevSweep[drawBufIdx] = true;

    pushFrame();
    uint32_t t3 = micros();
    lastFillUs = t1 - t0; lastDrawUs = t2 - t1; lastPushUs = t3 - t2;

    // Sync the OTHER framebuffer to the frame just presented. The two driver
    // buffers alternate on screen, and between full redraws only the sweep
    // line is updated (tracked per-buffer) — without this copy, every buffer
    // flip visibly bounces blips/labels/clock back and forth between the
    // states of the last two full redraws (~500ms apart). drawBufIdx has
    // already flipped inside pushFrame(), so panelFbs[drawBufIdx] is the
    // free buffer (waitFrameDone guaranteed it) and drawBufIdx^1 is the one
    // just presented. Reading a buffer mid-scan is safe; writing it wouldn't
    // be, which is why the copy direction matters.
    memcpy(panelFbs[drawBufIdx], panelFbs[drawBufIdx ^ 1], (size_t)LCD_W * LCD_H * sizeof(uint16_t));
    prevSweepX[drawBufIdx] = drawnSweepX;
    prevSweepY[drawBufIdx] = drawnSweepY;
    havePrevSweep[drawBufIdx] = true;

    lastSweep = now;
    return;
  }

  if (now - lastSweep < 30) return;
  lastSweep = now;

  float sweepDeg = fmodf(millis() / 15.0f, 360.0f);
  double sw = deg2rad(sweepDeg);
  int newX = L.cx + (int)(sin(sw) * L.R);
  int newY = L.cy - (int)(cos(sw) * L.R);

  if (havePrevSweep[drawBufIdx]) gfx->drawLine(L.cx, L.cy, prevSweepX[drawBufIdx], prevSweepY[drawBufIdx], BLACK);
  gfx->drawLine(L.cx, L.cy, newX, newY, GREEN);
  prevSweepX[drawBufIdx] = newX;
  prevSweepY[drawBufIdx] = newY;
  havePrevSweep[drawBufIdx] = true;

  pushFrame();
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

  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (!justEntered && now - lastDraw < 100) return;  // no animation on these screens, no need for a tight cadence
  uint32_t frameInterval = now - lastDraw;
  lastDraw = now;

  uint32_t t0 = micros();
  gfx->fillScreen(BLACK);
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
      prefs.putFloat("range", radarMaxKm);
      Serial.printf("[lcd7b] Range increased: %d km\n", (int)radarMaxKm);
      return;
    }
    if (sx >= bx && sx <= bx + bw && sy >= my && sy <= my + bh) {
      if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
      radarMaxKm = max(10.0f, radarMaxKm - 10.0f);
      if (configMutex) xSemaphoreGive(configMutex);
      prefs.putFloat("range", radarMaxKm);
      Serial.printf("[lcd7b] Range decreased: %d km\n", (int)radarMaxKm);
      return;
    }
  }

  // Default: cycle to next screen
  screen = (screen + 1) % LCD7B_NUM_SCREENS;
}
