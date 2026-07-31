/*
 * CYD28 (ESP32) Plane Spotter - Direct TFT Rendering
 * --------------------------------------------------------------------------
 * UPDATE: Merged Weather & System Status into a single unified screen.
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// =======================================================================
// CONFIGURATION — loaded from config.env
// =======================================================================
#include "config.env"

// Runtime-configurable location & radar (persisted via NVS)
// Defaults below; overridden by web UI and saved to flash.
double homeLat = DEFAULT_HOME_LAT;
double homeLon = DEFAULT_HOME_LON;
float  radarMaxKm = DEFAULT_RADAR_MAX_KM;
bool   invertColors = false;  // true = light theme (hardware color invert)
// Radar blip label field toggles — all default off, user opts in via web UI
bool   showCallsign = false;
bool   showSpeed = false;
bool   showFL = false;
bool   showRoute = false;
bool   showAirline = false;
bool   showReg = false;
bool   showSquawk = false;
bool   showVRate = false;
bool   showType = false;

SemaphoreHandle_t configMutex = NULL;  // protects homeLat/Lon/Radius/radarMaxKm
WebServer server(80);
Preferences prefs;
// =======================================================================

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

// Data models
struct Aircraft {
  char callsign[10]; char country[24];  // "country" repurposed to hold adsb.lol aircraft type code
  double lat; double lon; float altitudeM; float velocityMs;
  float trackDeg; float vrateMs; bool onGround; int category;
  double distanceKm; double bearingDeg; bool valid;
  char dep[5]; char arr[5]; bool hasRoute;
};
Aircraft nearest;

// Top 5 Tracker
const uint8_t MAX_TOP5 = 5;
Aircraft top5[MAX_TOP5];
uint8_t top5Count = 0;

// Blips for Radar
struct Blip {
  double lat; double lon; float track; float speedMs; char callsign[10]; float altitudeM;
  char airline[24]; char reg[10]; char squawk[6]; int16_t vrateFpm; char typeCode[6]; char category[3];
  char dep[4]; char arr[4]; bool hasRoute;  // dep/arr are IATA codes (from adsbdb.com)
};
const uint8_t MAX_BLIPS = 20;
Blip blips[MAX_BLIPS];
uint8_t blipCount = 0;
uint32_t lastDataMs = 0;

// Async data fetch (FreeRTOS task on Core 0)
SemaphoreHandle_t dataMutex = NULL;
TaskHandle_t fetchTaskHandle = NULL;
volatile bool newDataReady = false;
volatile bool fetchInProgress = false;

// Flight Info Cache — sized to comfortably hold routes for all MAX_BLIPS aircraft at once
#define ROUTE_CACHE_SIZE 24
struct CachedRoute {
  char callsign[10];
  char dep[4]; char arr[4];   // IATA codes, from adsbdb.com — already 3-letter, no ICAO conversion needed
  char airline[24];
  bool valid; uint32_t fetchTime;
};
CachedRoute routeCache[ROUTE_CACHE_SIZE];

struct Weather {
  float tempC; float windKmh; int humidity; int code; bool valid = false;
} weather;
uint32_t lastWeatherPoll = 0;

struct Stats {
  uint32_t requestsOk = 0; uint32_t requestsFail = 0;
  uint16_t inView = 0; uint16_t maxInView = 0;
  double closestEver = 1e9; uint32_t lastUpdateMs = 0;
} stats;

uint8_t screen = 0;
uint8_t lastScreen = 255;
uint32_t lastScreenSwap = 0;
uint32_t lastTouchMs = 0;
bool firstWeatherDone = false;
const uint8_t NUM_SCREENS = 4; // Reduced from 5 to 4 screens
const uint32_t SCREEN_SWAP_MS = 10000;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static double deg2rad(double d) { return d * (PI / 180.0); }
static double rad2deg(double r) { return r * (180.0 / PI); }

double haversineKm(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371.0;
  double dLat = deg2rad(lat2 - lat1); double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(deg2rad(lon2 - lon1)) * cos(deg2rad(lat2));
  double x = cos(deg2rad(lat1)) * sin(deg2rad(lat2)) - sin(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(lon2 - lon1));
  double b = rad2deg(atan2(y, x));
  return fmod(b + 360.0, 360.0);
}

const char* compass(double bearing) {
  static const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return dirs[(int)((bearing + 22.5) / 45.0) % 8];
}

void projectLatLon(double lat, double lon, float trackDeg, double distM, double& outLat, double& outLon) {
  double dr = distM / 6371000.0; double b = deg2rad(trackDeg);
  double la = deg2rad(lat), lo = deg2rad(lon);
  double nla = asin(sin(la) * cos(dr) + cos(la) * sin(dr) * cos(b));
  double nlo = lo + atan2(sin(b) * sin(dr) * cos(la), cos(dr) - sin(la) * sin(nla));
  outLat = rad2deg(nla); outLon = rad2deg(nlo);
}

// ---------------------------------------------------------------------------
// Network & APIs
// ---------------------------------------------------------------------------
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(4);
  tft.drawString("Connecting WiFi...", 20, 100);
  while (WiFi.status() != WL_CONNECTED) { delay(400); }
  tft.fillScreen(TFT_BLACK);
}

// adsbdb.com's /v0/callsign/{callsign} returns airline name + both airport IATA
// codes in one call — faster (~1s vs hexdb.io's 2-3s) and better data (real IATA
// codes globally, no ICAO->IATA guessing/static-table needed) than the two
// separate sources this used to combine.
bool getFlightInfo(const char* callsign, char* dep, char* arr, char* airline, bool allowFetch = true) {
  if (strlen(callsign) == 0 || strcmp(callsign, "(no id)") == 0) return false;

  for (int i=0; i<ROUTE_CACHE_SIZE; i++) {
    if (routeCache[i].valid && strcmp(routeCache[i].callsign, callsign) == 0) {
      if (routeCache[i].dep[0]) {
         strcpy(dep, routeCache[i].dep);
         strcpy(arr, routeCache[i].arr);
         strcpy(airline, routeCache[i].airline);
         return true;
      }
      return false;
    }
  }

  if (!allowFetch) return false;  // cache-only lookup — skip the live HTTP call

  WiFiClientSecure client; client.setInsecure();
  HTTPClient https; https.setReuse(false);
  String url = String("https://api.adsbdb.com/v0/callsign/") + callsign;
  bool found = false;
  dep[0] = '\0'; arr[0] = '\0'; airline[0] = '\0';

  if (https.begin(client, url)) {
    uint32_t t0 = millis();
    int code = https.GET();
    Serial.printf("[route] %s HTTP code=%d in %lums, heap=%u\n", callsign, code, millis() - t0, ESP.getFreeHeap());
    if (code == HTTP_CODE_OK) {
      String payload = https.getString();
      JsonDocument d;
      DeserializationError jerr = deserializeJson(d, payload);
      if (!jerr) {
        JsonObject fr = d["response"]["flightroute"];
        if (!fr.isNull()) {
          strncpy(dep, fr["origin"]["iata_code"] | "", 3); dep[3] = '\0';
          strncpy(arr, fr["destination"]["iata_code"] | "", 3); arr[3] = '\0';
          strncpy(airline, fr["airline"]["name"] | "", 23); airline[23] = '\0';
          found = dep[0] && arr[0];
        }
        Serial.printf("[route] %s -> %s/%s via %s found=%d\n", callsign, dep, arr, airline, found);
      } else {
        Serial.printf("[route] %s JSON parse failed: %s\n", callsign, jerr.c_str());
      }
    }
    https.end();
  } else {
    Serial.printf("[route] %s https.begin() failed\n", callsign);
  }

  int replaceIdx = 0;
  uint32_t oldest = 0xFFFFFFFF;
  for (int i=0; i<ROUTE_CACHE_SIZE; i++) {
    if (!routeCache[i].valid) { replaceIdx = i; break; }
    if (routeCache[i].fetchTime < oldest) { oldest = routeCache[i].fetchTime; replaceIdx = i; }
  }
  routeCache[replaceIdx].valid = true;
  strcpy(routeCache[replaceIdx].callsign, callsign);
  strcpy(routeCache[replaceIdx].dep, dep);
  strcpy(routeCache[replaceIdx].arr, arr);
  strcpy(routeCache[replaceIdx].airline, airline);
  routeCache[replaceIdx].fetchTime = millis();

  return found;
}

// Fetches aircraft data into caller-provided local buffers (thread-safe — no globals touched)
bool fetchAircraftTo(Aircraft* top5Out, uint8_t& top5CntOut,
                     Aircraft& nearOut, Blip* blipsOut, uint8_t& blipCntOut) {
  // Snapshot config under mutex (Core 0 reads, Core 1 may write via web UI)
  double hLat, hLon; float rMax;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; rMax = radarMaxKm;
  if (configMutex) xSemaphoreGive(configMutex);

  // adsb.lol's point endpoint takes a plain radius in nautical miles, which we
  // derive directly from the on-screen radar range — keeps fetch coverage and
  // display range in sync (adsb.lol caps this endpoint at 250nm).
  int radiusNm = (int)(rMax / 1.852f);
  if (radiusNm < 5) radiusNm = 5;
  if (radiusNm > 250) radiusNm = 250;

  WiFiClientSecure client; client.setInsecure();
  HTTPClient https; https.setReuse(false);
  char urlBuf[96];
  snprintf(urlBuf, sizeof(urlBuf), "https://api.adsb.lol/v2/point/%.4f/%.4f/%d", hLat, hLon, radiusNm);
  String url = urlBuf;
  Serial.printf("[fetch] GET %s (WiFi RSSI=%d dBm)\n", url.c_str(), WiFi.RSSI());

  if (!https.begin(client, url)) {
    Serial.printf("[fetch] https.begin() failed, heap=%u\n", ESP.getFreeHeap());
    stats.requestsFail++; return false;
  }
  uint32_t t0 = millis();
  int code = https.GET();
  Serial.printf("[fetch] HTTP code=%d in %lums, heap=%u\n", code, millis() - t0, ESP.getFreeHeap());
  if (code != HTTP_CODE_OK) { https.end(); stats.requestsFail++; return false; }

  JsonDocument filter;
  JsonObject fac = filter["ac"].to<JsonArray>().add<JsonObject>();
  fac["flight"] = true; fac["lat"] = true; fac["lon"] = true;
  fac["alt_baro"] = true; fac["gs"] = true; fac["track"] = true; fac["t"] = true;
  // Note: adsb.lol's "ownOp" field is confirmed null for essentially all traffic in
  // this airspace, so it's not requested — airline name now comes from adsbdb.com
  // via getFlightInfo() instead (see below), which actually has the data.
  fac["r"] = true; fac["squawk"] = true; fac["baro_rate"] = true; fac["category"] = true;

  String payload = https.getString(); https.end();
  Serial.printf("[fetch] payload=%u bytes, heap after read=%u\n", payload.length(), ESP.getFreeHeap());
  JsonDocument doc;
  DeserializationError jerr = deserializeJson(doc, payload, DeserializationOption::Filter(filter));
  if (jerr) {
    Serial.printf("[fetch] JSON parse failed: %s, heap=%u\n", jerr.c_str(), ESP.getFreeHeap());
    stats.requestsFail++; return false;
  }

  JsonArray ac = doc["ac"].as<JsonArray>();
  uint16_t count = 0;
  blipCntOut = 0;
  top5CntOut = 0;

  for (JsonObject obj : ac) {
    if (obj["lat"].isNull() || obj["lon"].isNull()) continue;

    // adsb.lol reports "alt_baro":"ground" (a string) for grounded aircraft —
    // skip those, matching the airborne-only behavior of the old OpenSky path.
    if (obj["alt_baro"].is<const char*>()) continue;

    double lat = obj["lat"].as<double>(), lon = obj["lon"].as<double>();
    double d = haversineKm(hLat, hLon, lat, lon);
    double brg = bearingDeg(hLat, hLon, lat, lon);
    count++;

    float altM = (obj["alt_baro"] | 0.0f) * 0.3048f;   // feet -> meters
    float spdMs = (obj["gs"] | 0.0f) * 0.514444f;      // knots -> m/s
    float trk = obj["track"] | 0.0f;

    if (blipCntOut < MAX_BLIPS) {
      blipsOut[blipCntOut].lat = lat; blipsOut[blipCntOut].lon = lon;
      blipsOut[blipCntOut].track = trk;
      blipsOut[blipCntOut].speedMs = spdMs;
      blipsOut[blipCntOut].altitudeM = altM;
      const char* cs = obj["flight"] | "";
      strncpy(blipsOut[blipCntOut].callsign, cs, 9);
      blipsOut[blipCntOut].callsign[9] = '\0';
      blipsOut[blipCntOut].airline[0] = '\0';  // filled in by getFlightInfo() below, if available
      strncpy(blipsOut[blipCntOut].reg, obj["r"] | "", 9);
      blipsOut[blipCntOut].reg[9] = '\0';
      strncpy(blipsOut[blipCntOut].squawk, obj["squawk"] | "", 5);
      blipsOut[blipCntOut].squawk[5] = '\0';
      blipsOut[blipCntOut].vrateFpm = (int16_t)(obj["baro_rate"] | 0.0f);
      strncpy(blipsOut[blipCntOut].typeCode, obj["t"] | "", 5);
      blipsOut[blipCntOut].typeCode[5] = '\0';
      strncpy(blipsOut[blipCntOut].category, obj["category"] | "", 2);
      blipsOut[blipCntOut].category[2] = '\0';
      blipsOut[blipCntOut].hasRoute = false;
      blipsOut[blipCntOut].dep[0] = '\0';
      blipsOut[blipCntOut].arr[0] = '\0';
      blipCntOut++;
    }

    Aircraft a;
    a.distanceKm = d; a.lat = lat; a.lon = lon; a.bearingDeg = brg;
    a.onGround = false; a.category = 0;
    a.altitudeM = altM;
    a.velocityMs = spdMs; a.trackDeg = trk; a.vrateMs = 0.0f;
    a.hasRoute = false; a.dep[0] = '\0'; a.arr[0] = '\0';
    a.valid = true;

    strncpy(a.callsign, obj["flight"] | "", 9); a.callsign[9] = '\0';
    for (int i = strlen(a.callsign) - 1; i >= 0 && a.callsign[i] == ' '; i--) a.callsign[i] = '\0';
    if (a.callsign[0] == '\0') strcpy(a.callsign, "(no id)");
    strncpy(a.country, obj["t"] | "?", 23); a.country[23] = '\0';  // aircraft type code, e.g. "B738"

    int insertIdx = -1;
    for (int i = 0; i < MAX_TOP5; i++) {
      if (i >= top5CntOut || a.distanceKm < top5Out[i].distanceKm) {
        insertIdx = i; break;
      }
    }
    if (insertIdx != -1) {
      for (int i = MAX_TOP5 - 1; i > insertIdx; i--) { top5Out[i] = top5Out[i-1]; }
      top5Out[insertIdx] = a;
      if (top5CntOut < MAX_TOP5) top5CntOut++;
    }
  }

  stats.inView = count; if (count > stats.maxInView) stats.maxInView = count;
  stats.requestsOk++;

  if (top5CntOut > 0) {
    nearOut = top5Out[0];
    if (nearOut.distanceKm < stats.closestEver)
      stats.closestEver = nearOut.distanceKm;
  } else {
    nearOut.valid = false;
  }

  return true;
}

// ===================================================================
// Background Data Fetcher — runs on Core 0 so Core 1 stays smooth
// ===================================================================
void dataFetcherTask(void* param) {
  // Local work buffers — all blocking HTTP happens here
  Aircraft locTop5[MAX_TOP5];
  Blip locBlips[MAX_BLIPS];
  uint8_t locTop5Count = 0;
  uint8_t locBlipCount = 0;
  Aircraft locNearest;
  locNearest.valid = false;

  for (;;) {
    // Wait for the next poll interval (sleep in 1s chunks so task is responsive)
    for (int t = 0; t < UPDATE_INTERVAL_MS / 1000; t++) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    fetchInProgress = true;
    bool ok = fetchAircraftTo(locTop5, locTop5Count, locNearest, locBlips, locBlipCount);
    fetchInProgress = false;

    if (ok) {
      // Fetch routes for the top 5 (quick per-call, but may still block a bit).
      // Aircraft doesn't currently display airline, so its name goes to a scratch buffer.
      char tmpAirline[24];
      for (int i = 0; i < locTop5Count; i++) {
        locTop5[i].hasRoute = getFlightInfo(locTop5[i].callsign, locTop5[i].dep, locTop5[i].arr, tmpAirline);
        locNearest = locTop5[0]; // nearest = closest after route resolution
      }

      // Flight info (route + airline) for all radar blips, without blocking the fetch
      // cycle for long: pass 1 is a free cache-only sweep (picks up anything already
      // known, including what the top-5 loop above just fetched); pass 2 spends a
      // small live-fetch budget on blips that are still uncached, so new aircraft
      // fill in over a couple of cycles instead of one fetch cycle blocking for up
      // to a minute.
      for (int i = 0; i < locBlipCount; i++) {
        locBlips[i].hasRoute = getFlightInfo(locBlips[i].callsign, locBlips[i].dep, locBlips[i].arr, locBlips[i].airline, false);
      }
      const int ROUTE_FETCH_BUDGET = 6;
      int routeFetchesUsed = 0;
      for (int i = 0; i < locBlipCount && routeFetchesUsed < ROUTE_FETCH_BUDGET; i++) {
        if (locBlips[i].hasRoute) continue;
        if (strlen(locBlips[i].callsign) == 0 || strcmp(locBlips[i].callsign, "(no id)") == 0) continue;
        locBlips[i].hasRoute = getFlightInfo(locBlips[i].callsign, locBlips[i].dep, locBlips[i].arr, locBlips[i].airline, true);
        routeFetchesUsed++;
      }

      // Atomically publish results to the globals used by render()
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      memcpy(blips, locBlips, sizeof(locBlips));
      blipCount = locBlipCount;
      memcpy(top5, locTop5, sizeof(locTop5));
      top5Count = locTop5Count;
      nearest = locNearest;
      lastDataMs = millis();
      stats.lastUpdateMs = lastDataMs;
      newDataReady = true;
      xSemaphoreGive(dataMutex);
    }
  }
}

bool fetchWeather() {
  // Snapshot config under mutex
  double hLat, hLon;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon;
  if (configMutex) xSemaphoreGive(configMutex);

  WiFiClientSecure client; client.setInsecure(); HTTPClient https; https.setReuse(false);
  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(hLat, 4) + "&longitude=" + String(hLon, 4) + "&current=temperature_2m,relative_humidity_2m,wind_speed_10m,weather_code";
  if (!https.begin(client, url)) { Serial.println("[weather] https.begin() failed"); return false; }
  uint32_t t0 = millis();
  int code = https.GET();
  Serial.printf("[weather] HTTP code=%d in %lums, heap=%u\n", code, millis() - t0, ESP.getFreeHeap());
  if (code != HTTP_CODE_OK) { https.end(); return false; }
  String payload = https.getString(); https.end();

  JsonDocument doc;
  DeserializationError jerr = deserializeJson(doc, payload);
  if (jerr) { Serial.printf("[weather] JSON parse failed: %s\n", jerr.c_str()); return false; }
  JsonObject c = doc["current"]; if (c.isNull()) { Serial.println("[weather] no 'current' object"); return false; }

  weather.tempC = c["temperature_2m"] | 0.0f; weather.humidity = c["relative_humidity_2m"] | 0;
  weather.windKmh = c["wind_speed_10m"] | 0.0f; weather.code = c["weather_code"] | 0; weather.valid = true;
  Serial.printf("[weather] temp=%.1fC humidity=%d%% wind=%.1fkm/h code=%d\n", weather.tempC, weather.humidity, weather.windKmh, weather.code);
  return true;
}

// ---------------------------------------------------------------------------
// UI & Drawing
// ---------------------------------------------------------------------------
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

// adsb.lol's "ownOp" registry field is null for the overwhelming majority of aircraft
// outside the US (confirmed empirically for this airspace — every aircraft in a live
// sample near Seoul came back with ownOp=null despite having callsigns, registrations,
// and type codes). Fall back to mapping the callsign's 3-letter ICAO airline prefix to
// a friendly name. Not exhaustive — unmapped prefixes just show nothing, same as today.
struct AirlineCode { const char* prefix; const char* name; };
static const AirlineCode AIRLINE_CODES[] = {
  // Korean carriers
  {"KAL", "Korean Air"}, {"AAR", "Asiana"}, {"JJA", "Jeju Air"}, {"ABL", "Air Busan"},
  {"ESR", "Eastar Jet"}, {"JNA", "Jin Air"}, {"TWB", "T'way Air"}, {"APZ", "Air Premia"},
  // Japanese / Chinese / other East Asian carriers
  {"ANA", "ANA"}, {"JAL", "JAL"}, {"CPA", "Cathay Pacific"}, {"HKE", "HK Express"},
  {"CES", "China Eastern"}, {"CCA", "Air China"}, {"CSN", "China Southern"},
  {"CHH", "Hainan Airlines"}, {"CXA", "Xiamen Air"}, {"EVA", "EVA Air"}, {"CAL", "China Airlines"},
  // Southeast Asian carriers
  {"SIA", "Singapore Air"}, {"THA", "Thai Airways"}, {"PAL", "Philippine Air"},
  {"MAS", "Malaysia Air"}, {"GIA", "Garuda Indonesia"}, {"VJC", "VietJet"}, {"HVN", "Vietnam Air"},
  // US carriers
  {"UAL", "United"}, {"DAL", "Delta"}, {"AAL", "American"}, {"FDX", "FedEx"}, {"UPS", "UPS"},
  // Middle East / Europe / other international
  {"QTR", "Qatar Airways"}, {"UAE", "Emirates"}, {"ETD", "Etihad"}, {"SVA", "Saudia"},
  {"THY", "Turkish Air"}, {"AFR", "Air France"}, {"DLH", "Lufthansa"}, {"BAW", "British Air"},
  {"KLM", "KLM"}, {"AFL", "Aeroflot"}, {"UZB", "Uzbekistan Air"},
};
static const int AIRLINE_CODES_COUNT = sizeof(AIRLINE_CODES) / sizeof(AIRLINE_CODES[0]);

const char* lookupAirline(const char* callsign) {
  if (strlen(callsign) < 3) return nullptr;
  for (int i = 0; i < AIRLINE_CODES_COUNT; i++) {
    if (strncmp(callsign, AIRLINE_CODES[i].prefix, 3) == 0) return AIRLINE_CODES[i].name;
  }
  return nullptr;
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
      int lineCount = 0;
      bool hasCallsign = blips[i].callsign[0] && strcmp(blips[i].callsign, "(no id)") != 0;

      if (showCS && hasCallsign) {
        strncpy(lines[lineCount], blips[i].callsign, 11); lines[lineCount][11] = '\0'; lineCount++;
      }
      if (showFlt) {
        float altFt = blips[i].altitudeM * 3.28084f;
        snprintf(lines[lineCount], 12, "FL%03.0f", altFt / 100.0f);
        lineCount++;
      }
      if (showRte && blips[i].hasRoute) {
        snprintf(lines[lineCount], 12, "%s>%s", blips[i].dep, blips[i].arr);
        lineCount++;
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
        snprintf(lines[lineCount], 12, "SQ%s", blips[i].squawk);
        lineCount++;
      }
      if (showVr) {
        snprintf(lines[lineCount], 12, "%+dfpm", blips[i].vrateFpm);
        lineCount++;
      }
      if (showTy && blips[i].typeCode[0]) {
        strncpy(lines[lineCount], blips[i].typeCode, 11); lines[lineCount][11] = '\0'; lineCount++;
      }
      if (showSpd) {
        float speedKt = blips[i].speedMs * 1.94384f;
        snprintf(lines[lineCount], 12, "%ukn", (unsigned int)(speedKt + 0.5f));
        lineCount++;
      }

      if (lineCount > 0) {
        int lx = bx + 4;
        if (lx + 54 > 200) lx = bx - 54;
        int totalH = lineCount * 8;
        int ly = by - 4;
        if (ly < 0) ly = 0;
        if (ly + totalH > 200) ly = 200 - totalH;
        if (ly < 0) ly = 0;

        for (int li = 0; li < lineCount; li++)
          radarSpr.drawString(lines[li], lx, ly + li * 8);
      }
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
    case 3: screenWeatherSystem(newPage); break;
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
            prefs.putFloat("range", radarMaxKm);
            Serial.printf("Range increased: %d km\n", (int)radarMaxKm);
            lastScreenSwap = millis();
            return;
          }
          // "-" button: sx=10..50, sy=206..240
          if (sx >= 10 && sx <= 50 && sy >= 206 && sy <= 240) {
            if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
            radarMaxKm = max(10.0f, radarMaxKm - 10.0f);
            if (configMutex) xSemaphoreGive(configMutex);
            prefs.putFloat("range", radarMaxKm);
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
// Web Server — Config Page
// ---------------------------------------------------------------------------
void initWebServer() {
  // GET / — serve config page
  server.on("/", []() {
    // Snapshot current values
    double hLat, hLon; float rMax; bool inv;
    bool showCS, showAir, showSpd, showFlt, showRte, showRg, showSq, showVr, showTy;
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    hLat = homeLat; hLon = homeLon; rMax = radarMaxKm; inv = invertColors;
    showCS = showCallsign; showAir = showAirline; showSpd = showSpeed; showFlt = showFL; showRte = showRoute;
    showRg = showReg; showSq = showSquawk; showVr = showVRate; showTy = showType;
    if (configMutex) xSemaphoreGive(configMutex);

    String html = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CYD Plane Spotter</title>
<style>
  body{font-family:Arial,sans-serif;background:#0a0a1a;color:#e0e0e0;margin:0;padding:20px}
  h1{color:#00ff88;text-align:center;font-size:1.4em}
  .card{background:#1a1a2e;border-radius:12px;padding:20px;margin:12px 0;max-width:420px;margin-left:auto;margin-right:auto}
  label{display:block;margin:10px 0 4px;color:#aaa;font-size:0.85em}
  input{width:100%;padding:10px;border:1px solid #333;border-radius:6px;background:#0d0d1f;color:#fff;font-size:1em;box-sizing:border-box}
  input:focus{border-color:#00ff88;outline:none}
  button{width:100%;padding:12px;margin-top:16px;background:#00ff88;color:#000;border:none;border-radius:8px;font-size:1.1em;font-weight:bold;cursor:pointer}
  button:hover{background:#00cc6a}
  .info{text-align:center;color:#666;font-size:0.8em;margin-top:12px}
  .saved{color:#00ff88;text-align:center;display:none;margin-top:8px}
  .switch-row{display:flex;align-items:center;justify-content:space-between;margin:14px 0 4px}
  .switch-row label{margin:0;color:#e0e0e0;font-size:1em}
  .switch{position:relative;display:inline-block;width:46px;height:24px;flex-shrink:0}
  .switch input{opacity:0;width:0;height:0}
  .slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#333;border-radius:24px;transition:.2s}
  .slider:before{position:absolute;content:"";height:18px;width:18px;left:3px;bottom:3px;background:#fff;border-radius:50%;transition:.2s}
  input:checked + .slider{background:#00ff88}
  input:checked + .slider:before{transform:translateX(22px)}
  .section-hd{color:#888;font-size:0.75em;text-transform:uppercase;letter-spacing:0.05em;margin:18px 0 4px;border-top:1px solid #333;padding-top:14px}
</style></head><body>
<h1>&#9992; CYD Plane Spotter</h1>
<div class="card">
  <form id="cfg" onsubmit="save(event)">
    <label>Home Latitude (&deg;)</label>
    <input type="number" step="any" name="lat" id="lat" value=")rawliteral" + String(hLat, 6) + R"rawliteral(" required>
    <label>Home Longitude (&deg;)</label>
    <input type="number" step="any" name="lon" id="lon" value=")rawliteral" + String(hLon, 6) + R"rawliteral(" required>
    <label>Radar Max Range (km) &mdash; also sets data fetch radius</label>
    <input type="number" step="any" name="range" id="range" value=")rawliteral" + String(rMax, 1) + R"rawliteral(" required>
    <div class="switch-row">
      <label for="dark">Dark Mode</label>
      <label class="switch">
        <input type="checkbox" id="dark")rawliteral" + String(inv ? "" : " checked") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="section-hd">Radar Label Fields</div>
    <div class="switch-row">
      <label for="callsign">Flight Number</label>
      <label class="switch">
        <input type="checkbox" id="callsign")rawliteral" + String(showCS ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="switch-row">
      <label for="airline">Airline Name</label>
      <label class="switch">
        <input type="checkbox" id="airline")rawliteral" + String(showAir ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="switch-row">
      <label for="speed">Speed (kn)</label>
      <label class="switch">
        <input type="checkbox" id="speed")rawliteral" + String(showSpd ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="switch-row">
      <label for="fl">Flight Level</label>
      <label class="switch">
        <input type="checkbox" id="fl")rawliteral" + String(showFlt ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="switch-row">
      <label for="route">Route (Src &rarr; Dest)</label>
      <label class="switch">
        <input type="checkbox" id="route")rawliteral" + String(showRte ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="switch-row">
      <label for="reg">Registration</label>
      <label class="switch">
        <input type="checkbox" id="reg")rawliteral" + String(showRg ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="switch-row">
      <label for="squawk">Squawk</label>
      <label class="switch">
        <input type="checkbox" id="squawk")rawliteral" + String(showSq ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="switch-row">
      <label for="vrate">Vertical Rate</label>
      <label class="switch">
        <input type="checkbox" id="vrate")rawliteral" + String(showVr ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="switch-row">
      <label for="type">Aircraft Type</label>
      <label class="switch">
        <input type="checkbox" id="type")rawliteral" + String(showTy ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <button type="submit">Save Settings</button>
  </form>
  <div class="saved" id="msg">&#10003; Settings saved!</div>
</div>
<div class="info">IP: )rawliteral" + WiFi.localIP().toString() + R"rawliteral(</div>
<script>
function save(e){
  e.preventDefault();
  var x=new XMLHttpRequest();
  x.open('POST','/save',true);
  x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');
  x.onload=function(){
    if(x.status==200){document.getElementById('msg').style.display='block';
    setTimeout(function(){document.getElementById('msg').style.display='none'},3000)}
  };
  x.send('lat='+encodeURIComponent(document.getElementById('lat').value)+
         '&lon='+encodeURIComponent(document.getElementById('lon').value)+
         '&range='+encodeURIComponent(document.getElementById('range').value)+
         '&dark='+(document.getElementById('dark').checked?'1':'0')+
         '&callsign='+(document.getElementById('callsign').checked?'1':'0')+
         '&airline='+(document.getElementById('airline').checked?'1':'0')+
         '&speed='+(document.getElementById('speed').checked?'1':'0')+
         '&fl='+(document.getElementById('fl').checked?'1':'0')+
         '&route='+(document.getElementById('route').checked?'1':'0')+
         '&reg='+(document.getElementById('reg').checked?'1':'0')+
         '&squawk='+(document.getElementById('squawk').checked?'1':'0')+
         '&vrate='+(document.getElementById('vrate').checked?'1':'0')+
         '&type='+(document.getElementById('type').checked?'1':'0'));
}
</script></body></html>)rawliteral";

    server.send(200, "text/html", html);
  });

  // POST /save — persist new config
  server.on("/save", []() {
    if (!server.hasArg("lat") || !server.hasArg("lon") || !server.hasArg("range")) {
      server.send(400, "text/plain", "Missing fields");
      return;
    }

    double newLat = server.arg("lat").toDouble();
    double newLon = server.arg("lon").toDouble();
    float  newRng = server.arg("range").toFloat();
    bool   newInvert = server.hasArg("dark") ? (server.arg("dark") != "1") : invertColors;
    bool   newShowCallsign = server.hasArg("callsign") ? (server.arg("callsign") == "1") : showCallsign;
    bool   newShowAirline = server.hasArg("airline") ? (server.arg("airline") == "1") : showAirline;
    bool   newShowSpeed = server.hasArg("speed") ? (server.arg("speed") == "1") : showSpeed;
    bool   newShowFL = server.hasArg("fl") ? (server.arg("fl") == "1") : showFL;
    bool   newShowRoute = server.hasArg("route") ? (server.arg("route") == "1") : showRoute;
    bool   newShowReg = server.hasArg("reg") ? (server.arg("reg") == "1") : showReg;
    bool   newShowSquawk = server.hasArg("squawk") ? (server.arg("squawk") == "1") : showSquawk;
    bool   newShowVRate = server.hasArg("vrate") ? (server.arg("vrate") == "1") : showVRate;
    bool   newShowType = server.hasArg("type") ? (server.arg("type") == "1") : showType;

    // Basic validation
    if (newLat < -90 || newLat > 90 || newLon < -180 || newLon > 180 ||
        newRng <= 0 || newRng > 500) {
      server.send(400, "text/plain", "Invalid values");
      return;
    }

    // Atomically update
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    homeLat = newLat;
    homeLon = newLon;
    radarMaxKm = newRng;
    invertColors = newInvert;
    showCallsign = newShowCallsign;
    showAirline = newShowAirline;
    showSpeed = newShowSpeed;
    showFL = newShowFL;
    showRoute = newShowRoute;
    showReg = newShowReg;
    showSquawk = newShowSquawk;
    showVRate = newShowVRate;
    showType = newShowType;
    if (configMutex) xSemaphoreGive(configMutex);
    tft.invertDisplay(newInvert);

    // Persist to NVS
    prefs.putDouble("lat", newLat);
    prefs.putDouble("lon", newLon);
    prefs.putFloat("range", newRng);
    prefs.putBool("invert", newInvert);
    prefs.putBool("callsign", newShowCallsign);
    prefs.putBool("airline", newShowAirline);
    prefs.putBool("speed", newShowSpeed);
    prefs.putBool("fl", newShowFL);
    prefs.putBool("route", newShowRoute);
    prefs.putBool("reg", newShowReg);
    prefs.putBool("squawk", newShowSquawk);
    prefs.putBool("vrate", newShowVRate);
    prefs.putBool("type", newShowType);

    Serial.printf("Config saved: lat=%.6f lon=%.6f range=%.1f invert=%d callsign=%d airline=%d speed=%d fl=%d route=%d reg=%d squawk=%d vrate=%d type=%d\n",
                  newLat, newLon, newRng, newInvert, newShowCallsign, newShowAirline, newShowSpeed, newShowFL,
                  newShowRoute, newShowReg, newShowSquawk, newShowVRate, newShowType);
    server.send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("Web server started on http://" + WiFi.localIP().toString() + "/");
}

// ---------------------------------------------------------------------------
// Main Arduino Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  touchSpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, -1);
  ts.begin(touchSpi); 
  ts.setRotation(1);

  tft.begin();
  tft.setRotation(1);

  radarSpr.setColorDepth(16);
  radarSpr.createSprite(200, 200);

  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(4); tft.setTextColor(TFT_GREEN);
  tft.drawString("CYD PLANE SPOTTER", 20, 100);
  delay(1500);

  for (int i=0; i<ROUTE_CACHE_SIZE; i++) routeCache[i].valid = false;

  connectWiFi();
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  setenv("TZ", TIMEZONE, 1); tzset();

  nearest.valid = false;
  newDataReady = false;
  fetchInProgress = false;

  // Load persisted config (or use defaults)
  prefs.begin("cyd-spotter", false);
  homeLat = prefs.getDouble("lat", homeLat);
  homeLon = prefs.getDouble("lon", homeLon);
  radarMaxKm = prefs.getFloat("range", radarMaxKm);
  invertColors = prefs.getBool("invert", invertColors);
  showCallsign = prefs.getBool("callsign", showCallsign);
  showAirline = prefs.getBool("airline", showAirline);
  showSpeed = prefs.getBool("speed", showSpeed);
  showFL = prefs.getBool("fl", showFL);
  showRoute = prefs.getBool("route", showRoute);
  showReg = prefs.getBool("reg", showReg);
  showSquawk = prefs.getBool("squawk", showSquawk);
  showVRate = prefs.getBool("vrate", showVRate);
  showType = prefs.getBool("type", showType);
  tft.invertDisplay(invertColors);
  Serial.printf("Config loaded: lat=%.6f lon=%.6f range=%.1f invert=%d callsign=%d airline=%d speed=%d fl=%d route=%d reg=%d squawk=%d vrate=%d type=%d\n",
                homeLat, homeLon, radarMaxKm, invertColors, showCallsign, showAirline, showSpeed, showFL,
                showRoute, showReg, showSquawk, showVRate, showType);

  // Create mutexes before the task (task uses them)
  dataMutex = xSemaphoreCreateMutex();
  configMutex = xSemaphoreCreateMutex();

  // Start web config server
  initWebServer();

  // Launch background fetcher on Core 0 (Core 1 = Arduino loop)
  xTaskCreatePinnedToCore(
    dataFetcherTask,   // Function
    "DataFetch",       // Name
    16384,             // Stack (16 KB — enough for JSON + SSL)
    NULL,              // Parameter
    1,                 // Priority
    &fetchTaskHandle,  // Handle
    0                  // Core 0 — leaves Core 1 free for rendering
  );
}

void loop() {
  uint32_t now = millis();

  // WiFi health — reconnect if needed (draws to TFT so must be Core 1)
  if (WiFi.status() != WL_CONNECTED) connectWiFi();

  // Weather fetch still runs in loop (it's infrequent, every 10 min)
  if (!firstWeatherDone || now - lastWeatherPoll >= WEATHER_INTERVAL_MS) {
    if (WiFi.status() == WL_CONNECTED) fetchWeather();
    lastWeatherPoll = now; firstWeatherDone = true;
  }

  checkTouch();

  // Serve web config requests
  server.handleClient();

  // Auto-screen rotation (re-enabled)
  //if (now - lastScreenSwap >= SCREEN_SWAP_MS) {
  //  screen = (screen + 1) % NUM_SCREENS;
  //  lastScreenSwap = now;
  //}

  render();
  delay(33);
}