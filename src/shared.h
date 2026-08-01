#pragma once
// =============================================================================
// Shared data layer: structs, config/state globals, math helpers, and the
// interface each board's display_*.cpp implements. No display-library types
// (TFT_eSPI, Arduino_GFX, etc) appear here — this header is included by every
// board's environment, so it can only depend on board-agnostic Arduino/ESP32
// APIs (WiFi, Preferences, WebServer, FreeRTOS).
// =============================================================================

#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Runtime-configurable location & radar (persisted via NVS)
// ---------------------------------------------------------------------------
extern double homeLat;
extern double homeLon;
extern float  radarMaxKm;
extern bool   invertColors;  // true = light theme (hardware color invert)
// Radar blip label field toggles — all default off, user opts in via web UI
extern bool   showCallsign;
extern bool   showSpeed;
extern bool   showFL;
extern bool   showRoute;
extern bool   showAirline;
extern bool   showReg;
extern bool   showSquawk;
extern bool   showVRate;
extern bool   showType;
// When true (default): if Airline is on but Route is off, resolve the airline name
// entirely from the local AIRLINE_CODES_FULL table and skip calling adsbdb.com for
// that aircraft — route data can't come from a static table (it's flight-specific,
// not a fixed code mapping), so a live call still happens whenever Route is on.
extern bool   preferLocalTables;

extern SemaphoreHandle_t configMutex;  // protects homeLat/Lon/radarMaxKm/toggles
extern WebServer server;
extern Preferences prefs;

// ---------------------------------------------------------------------------
// Data models
// ---------------------------------------------------------------------------
struct Aircraft {
  char callsign[10]; char country[24];  // "country" repurposed to hold adsb.lol aircraft type code
  double lat; double lon; float altitudeM; float velocityMs;
  float trackDeg; float vrateMs; bool onGround; int category;
  double distanceKm; double bearingDeg; bool valid;
  char dep[5]; char arr[5]; bool hasRoute;
};
extern Aircraft nearest;

const uint8_t MAX_TOP5 = 5;
extern Aircraft top5[MAX_TOP5];
extern uint8_t top5Count;

struct Blip {
  double lat; double lon; float track; float speedMs; char callsign[10]; float altitudeM;
  char airline[24]; char reg[10]; char squawk[6]; int16_t vrateFpm; char typeCode[6]; char category[3];
  char dep[4]; char arr[4]; bool hasRoute;  // dep/arr are IATA codes (from adsbdb.com)
};
const uint8_t MAX_BLIPS = 20;
extern Blip blips[MAX_BLIPS];
extern uint8_t blipCount;
extern uint32_t lastDataMs;

// Async data fetch (FreeRTOS task on Core 0)
extern SemaphoreHandle_t dataMutex;
extern TaskHandle_t fetchTaskHandle;
extern volatile bool newDataReady;
extern volatile bool fetchInProgress;

// Flight Info Cache — sized to comfortably hold routes for all MAX_BLIPS aircraft at once
#define ROUTE_CACHE_SIZE 24
struct CachedRoute {
  char callsign[10];
  char dep[4]; char arr[4];   // IATA codes, from adsbdb.com — already 3-letter, no ICAO conversion needed
  char airline[24];
  bool valid; uint32_t fetchTime;
};
extern CachedRoute routeCache[ROUTE_CACHE_SIZE];

struct Weather {
  float tempC; float windKmh; int humidity; int code; bool valid = false;
};
extern Weather weather;
extern uint32_t lastWeatherPoll;

struct Stats {
  uint32_t requestsOk = 0; uint32_t requestsFail = 0;
  uint16_t inView = 0; uint16_t maxInView = 0;
  double closestEver = 1e9; uint32_t lastUpdateMs = 0;
};
extern Stats stats;

extern uint8_t screen;
extern uint8_t lastScreen;
extern uint32_t lastScreenSwap;
extern uint32_t lastTouchMs;
extern bool firstWeatherDone;
const uint8_t NUM_SCREENS = 5; // Target Intel, Top 5, Radar PPI, Radar Full, Weather & System
const uint32_t SCREEN_SWAP_MS = 10000;

// ---------------------------------------------------------------------------
// Math helpers (pure, no display/network dependency — safe to define directly
// in this header since each including TU gets its own internal-linkage copy)
// ---------------------------------------------------------------------------
static double deg2rad(double d) { return d * (PI / 180.0); }
static double rad2deg(double r) { return r * (180.0 / PI); }

static double haversineKm(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371.0;
  double dLat = deg2rad(lat2 - lat1); double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

static double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(deg2rad(lon2 - lon1)) * cos(deg2rad(lat2));
  double x = cos(deg2rad(lat1)) * sin(deg2rad(lat2)) - sin(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(lon2 - lon1));
  double b = rad2deg(atan2(y, x));
  return fmod(b + 360.0, 360.0);
}

static const char* compass(double bearing) {
  static const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return dirs[(int)((bearing + 22.5) / 45.0) % 8];
}

static void projectLatLon(double lat, double lon, float trackDeg, double distM, double& outLat, double& outLon) {
  double dr = distM / 6371000.0; double b = deg2rad(trackDeg);
  double la = deg2rad(lat), lo = deg2rad(lon);
  double nla = asin(sin(la) * cos(dr) + cos(la) * sin(dr) * cos(b));
  double nlo = lo + atan2(sin(b) * sin(dr) * cos(la), cos(dr) - sin(la) * sin(nla));
  outLat = rad2deg(nla); outLon = rad2deg(nlo);
}

// ---------------------------------------------------------------------------
// Data layer, implemented once in data.cpp (board-agnostic)
// ---------------------------------------------------------------------------
void connectWiFi();  // pure network logic, no display calls — boards wrap this for visual feedback
bool getFlightInfo(const char* callsign, char* dep, char* arr, char* airline, bool allowFetch = true);
bool fetchAircraftTo(Aircraft* top5Out, uint8_t& top5CntOut,
                     Aircraft& nearOut, Blip* blipsOut, uint8_t& blipCntOut);
void dataFetcherTask(void* param);
bool fetchWeather();
const char* lookupAirline(const char* callsign);
void initWebServer();

// ---------------------------------------------------------------------------
// Display interface, implemented once per board in display_cyd.cpp /
// display_jc4832.cpp. main.cpp's setup()/loop() call only these — never any
// display-library type or function directly.
// ---------------------------------------------------------------------------
void displaySetup();               // touch/display/sprite/canvas init + splash screen
void connectWiFiShow();            // shows a "connecting" indicator, calls connectWiFi(), clears after
void applyInvertColors(bool invert);
void render();
void checkTouch();
