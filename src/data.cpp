/*
 * Shared data layer: WiFi, adsb.lol / adsbdb.com / Open-Meteo fetching, the
 * background fetch task, NVS-backed config globals, and the web config
 * server. No display-library code here — this file is compiled into every
 * board's environment unchanged.
 */

#include "shared.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Update.h>
#include "config.env"

// ---------------------------------------------------------------------------
// Global storage (declared extern in shared.h)
// ---------------------------------------------------------------------------
double homeLat = DEFAULT_HOME_LAT;
double homeLon = DEFAULT_HOME_LON;
float  radarMaxKm = DEFAULT_RADAR_MAX_KM;
bool   invertColors = false;
uint8_t brightness = 80;
bool   showCallsign = false;
bool   showSpeed = false;
bool   showFL = false;
bool   showRoute = false;
bool   showAirline = false;
bool   showReg = false;
bool   showSquawk = false;
bool   showVRate = false;
bool   showType = false;
bool   showTraces = false;
bool   preferLocalTables = true;

// Extended radar/display configuration — defaults match the previously
// hardcoded constants, so a fresh NVS behaves exactly like before.
// Colors are RGB565 (web UI converts to/from #rrggbb).
uint8_t  maxBlipsShown = MAX_BLIPS;
uint16_t trailMaxSamples = 1800;
uint16_t trailStaleSec = 15;
float    classNearKm = 25.0f;
uint16_t classMaxAltFt = 12000;
int16_t  classVrateFpm = 250;
float    sweepPeriodSec = 5.4f;
uint16_t radarRedrawMs = 500;
uint16_t fetchIntervalSec = UPDATE_INTERVAL_MS / 1000;
bool     showAirports = true;
bool     showTrailKey = true;
bool     showCompass = true;
bool     sweepGlow = true;
uint8_t  sweepGlowLen = 6;
bool     nightDimOn = false;
uint8_t  nightStartHr = 22;
uint8_t  nightEndHr = 7;
uint8_t  nightBrightPct = 15;

// Fixed day window for the weather widget (07:00-19:00 local). The
// configurable night-dim schedule is separate (nightStartHr/nightEndHr).
bool isNightNow() {
  if (time(nullptr) < 1700000000) return false;
  time_t t = time(nullptr); struct tm lt; localtime_r(&t, &lt);
  return lt.tm_hour >= 19 || lt.tm_hour < 7;
}
uint16_t colSweep = 0x07E0;      // GREEN
uint16_t colBlip = 0x07E0;       // GREEN
uint16_t colBlipHi = 0xFFE0;     // YELLOW
uint16_t colRings = 0x4208;      // DARKGREY
uint16_t colAirport = 0xF81F;    // MAGENTA
uint16_t colTrailDep = 0x07FF;   // CYAN
uint16_t colTrailArr = 0xFD20;   // ORANGE
uint16_t colTrailOver = 0xFFE0;  // YELLOW

FilterRule filterRules[FILTER_MAX_RULES];
bool filterQuiet = false;

SemaphoreHandle_t configMutex = NULL;
WebServer server(80);
Preferences prefs;

Aircraft nearest;
Aircraft top5[MAX_TOP5];
uint8_t top5Count = 0;

Blip blips[MAX_BLIPS];
uint8_t blipCount = 0;
uint32_t lastDataMs = 0;

SemaphoreHandle_t dataMutex = NULL;
TaskHandle_t fetchTaskHandle = NULL;
volatile bool newDataReady = false;
volatile bool fetchInProgress = false;

CachedRoute routeCache[ROUTE_CACHE_SIZE];

Weather weather;
uint32_t lastWeatherPoll = 0;

Stats stats;

uint8_t screen = 0;
uint8_t lastScreen = 255;
uint32_t lastScreenSwap = 0;
uint32_t lastTouchMs = 0;
bool firstWeatherDone = false;

// ---------------------------------------------------------------------------
// Network & APIs
// ---------------------------------------------------------------------------
WifiNet wifiNets[WIFI_MAX_NETWORKS];
bool wifiApFallbackActive = false;

// Loads the NVS network slots. Slot 0 is seeded from config.env's
// WIFI_SSID/WIFI_PASS if it has never been set, so existing installs keep
// working unchanged.
void wifiLoadNetworks() {
  for (uint8_t i = 0; i < WIFI_MAX_NETWORKS; i++) {
    char ks[10], kp[10];
    snprintf(ks, sizeof(ks), "wssid%d", i);
    snprintf(kp, sizeof(kp), "wpass%d", i);
    String s = prefs.getString(ks, "");
    String p = prefs.getString(kp, "");
    if (i == 0 && s.length() == 0) {
      s = WIFI_SSID; p = WIFI_PASS;
      prefs.putString(ks, s); prefs.putString(kp, p);
    }
    strncpy(wifiNets[i].ssid, s.c_str(), 32); wifiNets[i].ssid[32] = '\0';
    strncpy(wifiNets[i].pass, p.c_str(), 64); wifiNets[i].pass[64] = '\0';
  }
}

// Deferred NVS persistence for high-frequency config changes (the zoom
// buttons). Writing NVS means a flash erase/write with the CPU cache
// disabled, and on the LCD-7B v2 build the RGB driver's bounce-buffer
// refill ISR is not IRAM-safe — an NVS write in the touch path visibly
// glitches/desyncs the panel. So interactive code marks the value dirty
// here, and configMaintain() (called from loop(), when the user has
// stopped pressing buttons for 3s) does the actual write once.
static volatile bool rangeDirty = false;
static uint32_t rangeDirtyMs = 0;
void markRangeDirty() {
  rangeDirty = true;
  rangeDirtyMs = millis();
}
void configMaintain() {
  if (rangeDirty && millis() - rangeDirtyMs > 3000) {
    rangeDirty = false;
    float r;
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    r = radarMaxKm;
    if (configMutex) xSemaphoreGive(configMutex);
    prefs.putFloat("range", r);
    Serial.printf("[cfg] range %.0f km persisted (deferred)\n", r);
  }
}

static bool trySlot(uint8_t slot, uint32_t timeoutMs) {
  if (!wifiNets[slot].ssid[0]) return false;
  Serial.printf("[wifi] trying '%s'...\n", wifiNets[slot].ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiNets[slot].ssid, wifiNets[slot].pass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) delay(250);
  bool ok = WiFi.status() == WL_CONNECTED;
  Serial.printf("[wifi] '%s': %s\n", wifiNets[slot].ssid, ok ? "connected" : "timeout");
  return ok;
}

static void startFallbackAp() {
  if (wifiApFallbackActive) return;
  Serial.println("[wifi] no saved network reachable — starting setup AP 'PlaneSpotter-Setup'");
  WiFi.mode(WIFI_AP_STA);  // AP stays up for the config page while STA retries continue
  WiFi.softAP("PlaneSpotter-Setup");
  wifiApFallbackActive = true;
}

// Boot-time connect: two full rounds over all saved networks, then the
// fallback setup AP. Returns whenever connected (or after AP start) — never
// blocks forever, so boot always completes.
void connectWiFi() {
  for (uint8_t round = 0; round < 2 && WiFi.status() != WL_CONNECTED; round++) {
    for (uint8_t i = 0; i < WIFI_MAX_NETWORKS; i++) {
      if (trySlot(i, 8000)) {
        Serial.printf("[wifi] connected, IP=%s\n", WiFi.localIP().toString().c_str());
        return;
      }
    }
  }
  startFallbackAp();
}

// Non-blocking WiFi maintenance, called every loop() iteration. Keeps the
// radar running on stale data during outages instead of freezing the UI on a
// blocking reconnect. Round-robins saved networks with backoff; after two
// full failed cycles starts the setup AP (WIFI_AP_STA, so the config page is
// reachable at 192.168.4.1 while retries continue in the background).
void wifiMaintain() {
  enum : uint8_t { WS_OK, WS_CONNECTING, WS_WAIT };
  static uint8_t state = WS_OK;
  static uint8_t slot = 0;
  static uint32_t t = 0;
  static uint8_t failRounds = 0;
  uint32_t now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    if (state != WS_OK) {
      state = WS_OK; failRounds = 0;
      Serial.printf("[wifi] connected, IP=%s\n", WiFi.localIP().toString().c_str());
    }
    if (wifiApFallbackActive) {  // back online: tear down the setup AP
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      wifiApFallbackActive = false;
      Serial.println("[wifi] setup AP stopped");
    }
    return;
  }

  switch (state) {
    case WS_OK:  // just lost the connection
      state = WS_WAIT; t = now;
      break;
    case WS_WAIT:
      if (now - t < 3000) break;
      for (uint8_t k = 0; k < WIFI_MAX_NETWORKS; k++) {  // next configured slot
        if (wifiNets[slot].ssid[0]) break;
        slot = (slot + 1) % WIFI_MAX_NETWORKS;
      }
      if (!wifiNets[slot].ssid[0]) { startFallbackAp(); state = WS_WAIT; t = now; break; }
      WiFi.disconnect(false);
      WiFi.mode(wifiApFallbackActive ? WIFI_AP_STA : WIFI_STA);
      WiFi.begin(wifiNets[slot].ssid, wifiNets[slot].pass);
      Serial.printf("[wifi] reconnect: trying '%s'\n", wifiNets[slot].ssid);
      state = WS_CONNECTING; t = now;
      break;
    case WS_CONNECTING:
      if (now - t < 7000) break;
      slot = (slot + 1) % WIFI_MAX_NETWORKS;
      if (++failRounds >= 2) startFallbackAp();
      state = WS_WAIT; t = now;
      break;
  }
}

// adsbdb.com's /v0/callsign/{callsign} returns airline name + both airport IATA
// codes in one call — faster (~1s vs hexdb.io's 2-3s) and better data (real IATA
// codes globally, no ICAO->IATA guessing/static-table needed) than the two
// separate sources this used to combine.
bool getFlightInfo(const char* callsign, char* dep, char* arr, char* airline, bool allowFetch) {
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
    // Wait for the next poll interval (sleep in 1s chunks so task is responsive).
    // Interval is runtime-configurable via the web UI (fetchIntervalSec).
    uint16_t intervalSec;
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    intervalSec = fetchIntervalSec;
    if (configMutex) xSemaphoreGive(configMutex);
    for (uint16_t t = 0; t < intervalSec; t++) {
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

      // Flight info (route + airline) for radar blips — but only bother at all if
      // Route or Airline is actually toggled on; nothing to fetch otherwise.
      bool needRte, needAir, preferTables;
      if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
      needRte = showRoute; needAir = showAirline; preferTables = preferLocalTables;
      if (configMutex) xSemaphoreGive(configMutex);

      if (needRte || needAir) {
        if (!needRte && preferTables) {
          // Route not needed, and tables are preferred: route is inherently live,
          // per-flight data no static table can provide, but airline name can be —
          // resolve entirely locally, skip calling adsbdb.com for this pass altogether.
          for (int i = 0; i < locBlipCount; i++) {
            const char* a = lookupAirline(locBlips[i].callsign);
            if (a) { strncpy(locBlips[i].airline, a, 23); locBlips[i].airline[23] = '\0'; }
            else locBlips[i].airline[0] = '\0';
            locBlips[i].hasRoute = false;
          }
        } else {
          // Route is needed (or tables aren't preferred): must hit adsbdb.com. Pass 1
          // is a free cache-only sweep (picks up anything already known, including
          // what the top-5 loop above just fetched); pass 2 spends a small live-fetch
          // budget on blips that are still uncached, so new aircraft fill in over a
          // couple of cycles instead of one fetch cycle blocking for up to a minute.
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
        }
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


struct AirlineCode2 { const char* prefix; const char* name; };
static const AirlineCode2 AIRLINE_CODES_FULL[] = {
  // Korea
  {"KAL", "Korean Air"}, {"AAR", "Asiana"}, {"JJA", "Jeju Air"}, {"ABL", "Air Busan"}, {"ESR", "Eastar Jet"},
  {"JNA", "Jin Air"}, {"TWB", "T'way Air"}, {"APZ", "Air Premia"},
  // Japan / China / Taiwan / Hong Kong / Mongolia / N. Korea
  {"ANA", "ANA"}, {"JAL", "JAL"}, {"CPA", "Cathay Pacific"}, {"HKE", "HK Express"}, {"CES", "China Eastern"},
  {"CCA", "Air China"}, {"CSN", "China Southern"}, {"CHH", "Hainan Airlines"}, {"CXA", "Xiamen Air"},
  {"EVA", "EVA Air"}, {"CAL", "China Airlines"}, {"CSZ", "Shenzhen Air"}, {"CSC", "Sichuan Airlines"},
  {"CDG", "Shandong Air"}, {"CQH", "Spring Airlines"}, {"CUA", "China United"}, {"DKH", "Juneyao Air"},
  {"GCR", "Tianjin Airlines"}, {"CBJ", "Beijing Capital"}, {"CDC", "Loong Air"}, {"OKA", "Okay Airways"},
  {"RLH", "Ruili Airlines"}, {"CYZ", "China Postal"}, {"CSS", "SF Airlines"}, {"CKK", "China Cargo"},
  {"AHK", "Air Hong Kong"}, {"HGB", "Greater Bay"}, {"AMU", "Air Macau"}, {"UIA", "Uni Air"}, {"SJX", "Starlux"},
  {"TTW", "Tigerair TW"}, {"APJ", "Peach"}, {"JJP", "Jetstar Japan"}, {"TZP", "ZIPAIR"}, {"SNJ", "Solaseed Air"},
  {"SKY", "Skymark"}, {"SFJ", "StarFlyer"}, {"MGL", "MIAT Mongolia"}, {"KOR", "Air Koryo"},
  // Southeast Asia
  {"SIA", "Singapore Air"}, {"THA", "Thai Airways"}, {"PAL", "Philippine Air"}, {"MAS", "Malaysia Air"},
  {"GIA", "Garuda Indonesia"}, {"VJC", "VietJet"}, {"HVN", "Vietnam Air"}, {"TGW", "Scoot"}, {"JSA", "Jetstar Asia"},
  {"AIQ", "Thai AirAsia"}, {"TLM", "Thai Lion Air"}, {"BKP", "Bangkok Air"}, {"NOK", "Nok Air"},
  {"AXM", "AirAsia"}, {"XAX", "AirAsia X"}, {"FFM", "Firefly"}, {"CEB", "Cebu Pacific"}, {"GAP", "PAL Express"},
  {"LNI", "Lion Air"}, {"SJY", "Sriwijaya Air"}, {"BAV", "Bamboo Airways"}, {"PIC", "Pacific Airlines"},
  {"VAG", "Vietravel Air"}, {"KME", "Cambodia Airways"}, {"LAO", "Lao Airlines"}, {"MMA", "Myanmar Airways"},
  {"GMR", "Golden Myanmar"}, {"UBA", "Myanmar National"}, {"RBA", "Royal Brunei"}, {"BTK", "Batik Air"},
  {"CTV", "Citilink"},
  // South Asia
  {"AIC", "Air India"}, {"AXB", "Air India Express"}, {"IGO", "IndiGo"}, {"SEJ", "SpiceJet"}, {"AKJ", "Akasa Air"},
  {"PIA", "Pakistan Intl"}, {"ABQ", "Airblue"}, {"BBC", "Biman Bangladesh"}, {"UBG", "US-Bangla"},
  {"ALK", "SriLankan"}, {"EXV", "FitsAir"}, {"RNA", "Nepal Airlines"}, {"BHA", "Buddha Air"}, {"DRK", "Druk Air"},
  {"DQA", "Maldivian"},
  // Central Asia / Caucasus
  {"KZR", "Air Astana"}, {"VSV", "SCAT Airlines"}, {"UZB", "Uzbekistan Air"}, {"SMR", "Somon Air"},
  {"TUA", "Turkmenistan Air"}, {"AVJ", "Avia Traffic"}, {"TGZ", "Georgian Airways"}, {"AHY", "Azerbaijan Air"},
  {"FIA", "FlyOne"}, {"FIE", "FlyOne Armenia"},
  // Middle East
  {"QTR", "Qatar Airways"}, {"UAE", "Emirates"}, {"ETD", "Etihad"}, {"SVA", "Saudia"}, {"THY", "Turkish Air"},
  {"KNE", "flynas"}, {"FAD", "flyadeal"}, {"RJA", "Royal Jordanian"}, {"MEA", "MEA"}, {"KAC", "Kuwait Airways"},
  {"JZR", "Jazeera Airways"}, {"GFA", "Gulf Air"}, {"OMA", "Oman Air"}, {"OMS", "SalamAir"}, {"ABY", "Air Arabia"},
  {"FDB", "flydubai"}, {"ELY", "El Al"}, {"ISR", "Israir"}, {"AIZ", "Arkia"}, {"IAW", "Iraqi Airways"},
  {"IRA", "Iran Air"}, {"IRM", "Mahan Air"}, {"IYE", "Yemenia"}, {"WAN", "Wataniya"},
  // Africa
  {"ETH", "Ethiopian"}, {"KQA", "Kenya Airways"}, {"SAA", "South African"}, {"MSR", "EgyptAir"},
  {"RAM", "Royal Air Maroc"}, {"TAR", "Tunisair"}, {"DAH", "Air Algerie"}, {"ARA", "Arik Air"},
  {"AFW", "Africa World"}, {"RWD", "RwandAir"}, {"ATC", "Air Tanzania"}, {"PRF", "Precision Air"},
  {"UGD", "Uganda Airlines"}, {"NMB", "Air Namibia"}, {"CAW", "Comair"}, {"SFR", "Safair"}, {"MNO", "Mango"},
  {"DTA", "TAAG Angola"}, {"LAM", "LAM Mozambique"}, {"AZW", "Air Zimbabwe"}, {"BOT", "Air Botswana"},
  {"SEY", "Air Seychelles"}, {"MAU", "Air Mauritius"}, {"MDG", "Air Madagascar"}, {"TDS", "Tsaradia"},
  {"CRC", "Camair-Co"}, {"SZN", "Air Senegal"}, {"VRE", "Air Cote d'Ivoire"}, {"VBW", "Air Burkina"},
  {"LYW", "Libyan Airlines"}, {"AAW", "Afriqiyah"}, {"SUD", "Sudan Airways"}, {"FTZ", "Fastjet"},
  {"OLA", "Overland Air"}, {"LNK", "Airlink"},
  // South America
  {"CMP", "Copa Airlines"}, {"AVA", "Avianca"}, {"GLG", "Avianca Ecuador"}, {"AZU", "Azul"}, {"GLO", "Gol"},
  {"SKU", "Sky Airline"}, {"JAT", "JetSmart"}, {"JAP", "JetSmart Peru"}, {"ARG", "Aerolineas Arg"},
  {"FBZ", "Flybondi"}, {"AZN", "Amaszonas"}, {"BOV", "Boliviana"}, {"VCV", "Conviasa"}, {"NSE", "Satena"},
  {"DSM", "LATAM Argentina"}, {"TAM", "LATAM Brasil"}, {"LAN", "LATAM Chile"}, {"LNE", "LATAM Ecuador"},
  {"LAP", "LATAM Paraguay"}, {"LPE", "LATAM Peru"}, {"ULS", "Ultra Air"},
  // Caribbean / Central America
  {"BWA", "Caribbean Air"}, {"BHS", "Bahamasair"}, {"CAY", "Cayman Airways"}, {"IWY", "InterCaribbean"},
  {"ARU", "Aruba Airlines"}, {"SLM", "Surinam Airways"}, {"CUB", "Cubana"}, {"AJM", "Air Jamaica"},
  {"RPB", "Wingo"},
  // Europe
  {"AFR", "Air France"}, {"DLH", "Lufthansa"}, {"BAW", "British Air"}, {"KLM", "KLM"}, {"AFL", "Aeroflot"},
  {"IBE", "Iberia"}, {"ITY", "ITA Airways"}, {"TAP", "TAP Portugal"}, {"SAS", "SAS"}, {"FIN", "Finnair"},
  {"SWR", "Swiss"}, {"AUA", "Austrian"}, {"BEL", "Brussels Air"}, {"LOT", "LOT Polish"}, {"CSA", "Czech Airlines"},
  {"CTN", "Croatia Air"}, {"AEE", "Aegean"}, {"OAL", "Olympic Air"}, {"NOZ", "Norwegian"}, {"ICE", "Icelandair"},
  {"BTI", "airBaltic"}, {"AUI", "Ukraine Intl"}, {"WZZ", "Wizz Air"}, {"RYR", "Ryanair"}, {"EZY", "easyJet"},
  {"VLG", "Vueling"}, {"EWG", "Eurowings"}, {"CFG", "Condor"}, {"TUI", "TUIfly"}, {"PGT", "Pegasus"},
  {"SXS", "SunExpress"}, {"CRL", "Corsair"}, {"AEA", "Air Europa"}, {"VOE", "Volotea"}, {"TRA", "Transavia"},
  {"EIN", "Aer Lingus"}, {"LGL", "Luxair"}, {"ROT", "Tarom"}, {"LZB", "Bulgaria Air"}, {"ASL", "Air Serbia"},
  {"MNE", "Air Montenegro"}, {"CYP", "Cyprus Airways"}, {"SBI", "S7 Airlines"}, {"SVR", "Ural Airlines"},
  {"NWS", "Nordwind"}, {"PBD", "Pobeda"}, {"BRU", "Belavia"}, {"MLD", "Air Moldova"}, {"TVS", "Smartwings"},
  {"NBT", "Norse Atlantic"}, {"GRL", "Air Greenland"}, {"FLI", "Atlantic Airways"}, {"CLX", "Cargolux"},
  {"VDA", "Volga-Dnepr"}, {"ADB", "Antonov Air"}, {"DHK", "DHL Air"}, {"MPH", "Martinair"},
  // North America
  {"AAL", "American"}, {"UAL", "United"}, {"DAL", "Delta"}, {"SWA", "Southwest"}, {"JBU", "JetBlue"},
  {"ASA", "Alaska"}, {"NKS", "Spirit"}, {"FFT", "Frontier"}, {"HAL", "Hawaiian"}, {"AAY", "Allegiant"},
  {"FDX", "FedEx"}, {"UPS", "UPS"}, {"GTI", "Atlas Air"}, {"SCX", "Sun Country"}, {"ENY", "Envoy Air"},
  {"SKW", "SkyWest"}, {"RPA", "Republic"}, {"EDV", "Endeavor Air"}, {"ASH", "Mesa Airlines"}, {"SIL", "Silver Airways"},
  {"MXY", "Breeze"}, {"PAC", "Polar Air Cargo"}, {"CKS", "Kalitta Air"}, {"ACA", "Air Canada"},
  {"ROU", "Air Canada Rouge"}, {"WJA", "WestJet"}, {"TSC", "Air Transat"}, {"FLE", "Flair"}, {"SWG", "Sunwing"},
  {"CJT", "Cargojet"}, {"AMX", "Aeromexico"}, {"VOI", "Volaris"}, {"TAO", "Aeromar"}, {"VIV", "VivaAerobus"},
  {"PTR", "Porter Airlines"}, {"AJT", "Amerijet"}, {"ABX", "ABX Air"}, {"WGN", "Western Global"},
  // Oceania
  {"QFA", "Qantas"}, {"JST", "Jetstar"}, {"VOZ", "Virgin Australia"}, {"RXA", "Rex Airlines"}, {"ANZ", "Air New Zealand"},
  {"FJI", "Fiji Airways"}, {"AVN", "Air Vanuatu"}, {"ANG", "Air Niugini"}, {"SOL", "Solomon Airlines"},
  {"THT", "Air Tahiti Nui"}, {"VTA", "Air Tahiti"}, {"ACI", "Aircalin"}, {"RON", "Nauru Airlines"},
  {"AKL", "Air Kiribati"}, {"RAR", "Air Rarotonga"}, {"CVA", "Air Chathams"},
};
static const int AIRLINE_CODES_FULL_COUNT = sizeof(AIRLINE_CODES_FULL) / sizeof(AIRLINE_CODES_FULL[0]);

// adsb.lol's "ownOp" registry field is null for the overwhelming majority of aircraft
// outside the US (confirmed empirically for this airspace — every aircraft in a live
// sample near Seoul came back with ownOp=null despite having callsigns, registrations,
// and type codes), and adsbdb.com's live callsign lookup is a per-aircraft network
// call. lookupAirline() maps a callsign's 3-letter ICAO prefix to a friendly airline
// name entirely locally — used both as a fallback when adsbdb.com has no data, and
// (when preferLocalTables is on and Route isn't needed) to skip calling adsbdb.com
// for airline name entirely. Table is AIRLINE_CODES_FULL below (303 entries, sourced
// from raw Wikipedia wikitext, cross-checked entry by entry — not exhaustive, but
// covers essentially all scheduled passenger/cargo carriers worldwide).

const char* lookupAirline(const char* callsign) {
  if (strlen(callsign) < 3) return nullptr;
  for (int i = 0; i < AIRLINE_CODES_FULL_COUNT; i++) {
    if (strncmp(callsign, AIRLINE_CODES_FULL[i].prefix, 3) == 0) return AIRLINE_CODES_FULL[i].name;
  }
  return nullptr;
}

// =============================================================================
// AIRPORT_CODES_ICAO_IATA below (319 entries, ~5.3KB, ICAO->IATA airport code
// conversion, same verification process) has no current use case at all: the
// only place that used to need ICAO->IATA conversion was the old hexdb.io-based
// route lookup, which has been replaced by adsbdb.com's getFlightInfo() — that
// already returns real IATA codes directly, globally, so there's no ICAO code
// left anywhere in the data path to convert. It would only become useful again
// if a secondary/backup route data source were added for when adsbdb.com is
// unreachable or doesn't recognize a callsign.
// =============================================================================


struct AirportCode2 { const char* icao; const char* iata; };
static const AirportCode2 AIRPORT_CODES_ICAO_IATA[] = {
  // Korea
  {"RKSI", "ICN"}, {"RKSS", "GMP"}, {"RKPC", "CJU"}, {"RKPK", "PUS"}, {"RKNY", "YNY"},
  // Japan
  {"RJAA", "NRT"}, {"RJTT", "HND"}, {"RJGG", "NGO"}, {"RJBB", "KIX"}, {"RJCC", "CTS"}, {"RJFF", "FUK"},
  {"RJOO", "ITM"}, {"RJSS", "SDJ"}, {"RJOA", "HIJ"}, {"ROAH", "OKA"}, {"RJFK", "KOJ"}, {"RJBE", "UKB"},
  // China
  {"ZBAA", "PEK"}, {"ZBAD", "PKX"}, {"ZSPD", "PVG"}, {"ZSSS", "SHA"}, {"ZGGG", "CAN"}, {"ZGSZ", "SZX"},
  {"ZUUU", "CTU"}, {"ZPPP", "KMG"}, {"ZHHH", "WUH"}, {"ZSAM", "XMN"}, {"ZSNJ", "NKG"}, {"ZLXY", "XIY"},
  {"ZSHC", "HGH"}, {"ZUCK", "CKG"}, {"ZYTX", "SHE"}, {"ZYHB", "HRB"}, {"ZBTJ", "TSN"}, {"ZSQD", "TAO"},
  {"ZGHA", "CSX"}, {"ZHCC", "CGO"}, {"ZUGY", "KWE"}, {"ZLLL", "LHW"}, {"ZWWW", "URC"}, {"ZJSY", "SYX"},
  {"ZSFZ", "FOC"}, {"ZGNN", "NNG"},
  // Hong Kong / Taiwan / Macau
  {"VHHH", "HKG"}, {"RCTP", "TPE"}, {"RCSS", "TSA"}, {"VMMC", "MFM"},
  // Mongolia / North Korea
  {"ZMUB", "ULN"}, {"ZKPY", "FNJ"},
  // Southeast Asia
  {"WSSS", "SIN"}, {"VTBS", "BKK"}, {"VTBD", "DMK"}, {"WMKK", "KUL"}, {"RPLL", "MNL"}, {"RPVM", "CEB"},
  {"WIII", "CGK"}, {"WARR", "SUB"}, {"WADD", "DPS"}, {"VVNB", "HAN"}, {"VVTS", "SGN"}, {"VVDN", "DAD"},
  {"VLVT", "VTE"}, {"VDPP", "PNH"}, {"VYYY", "RGN"}, {"VYMD", "MDL"}, {"WBSB", "BWN"}, {"WMKP", "PEN"},
  {"WMKJ", "JHB"}, {"WBKK", "BKI"}, {"VTSP", "HKT"}, {"VTCC", "CNX"},
  // South Asia
  {"VIDP", "DEL"}, {"VABB", "BOM"}, {"VOBL", "BLR"}, {"VOMM", "MAA"}, {"VECC", "CCU"}, {"VOHY", "HYD"},
  {"VOCI", "COK"}, {"VAAH", "AMD"}, {"VAGO", "GOI"}, {"OPKC", "KHI"}, {"OPLA", "LHE"}, {"OPIS", "ISB"},
  {"VGHS", "DAC"}, {"VCBI", "CMB"}, {"VNKT", "KTM"}, {"VRMM", "MLE"},
  // Central Asia / Caucasus
  {"UAAA", "ALA"}, {"UACC", "NQZ"}, {"UCFM", "BSZ"}, {"UTTT", "TAS"}, {"UTDD", "DYU"}, {"UTAA", "ASB"},
  {"UGTB", "TBS"}, {"UBBB", "GYD"}, {"UDYZ", "EVN"},
  // Middle East
  {"OTHH", "DOH"}, {"OMDB", "DXB"}, {"OMAA", "AUH"}, {"OMSJ", "SHJ"}, {"OMDW", "DWC"}, {"OERK", "RUH"},
  {"OEJN", "JED"}, {"OEMA", "MED"}, {"OEDF", "DMM"}, {"OKKK", "KWI"}, {"OBBI", "BAH"}, {"OOMS", "MCT"},
  {"OOSA", "SLL"}, {"OJAI", "AMM"}, {"OLBA", "BEY"}, {"ORBI", "BGW"}, {"ORER", "EBL"}, {"OIIE", "IKA"},
  {"OIII", "THR"}, {"OISS", "SYZ"}, {"OIFM", "IFN"}, {"OITT", "TBZ"}, {"OIMM", "MHD"}, {"LLBG", "TLV"},
  {"LLER", "ETM"}, {"OYSN", "SAH"}, {"OYAA", "ADE"},
  // Africa
  {"HECA", "CAI"}, {"HAAB", "ADD"}, {"HKJK", "NBO"}, {"HKMO", "MBA"}, {"HTDA", "DAR"}, {"HTZA", "ZNZ"},
  {"HUEN", "EBB"}, {"HSSK", "KRT"}, {"HDAM", "JIB"}, {"HCMM", "MGQ"}, {"HHAS", "ASM"}, {"HJJJ", "JUB"},
  {"FACT", "CPT"}, {"FAOR", "JNB"}, {"FALE", "DUR"}, {"FAPE", "PLZ"}, {"FABL", "BFN"}, {"DNMM", "LOS"},
  {"DNAA", "ABV"}, {"GMMN", "CMN"}, {"GMME", "RBA"}, {"GMMX", "RAK"}, {"DTTA", "TUN"}, {"DAAG", "ALG"},
  {"GOBD", "DSS"}, {"GABS", "BKO"}, {"GUCY", "CKY"}, {"GFLL", "FNA"}, {"GLRB", "ROB"}, {"GBYD", "BJL"},
  {"GGOV", "OXB"}, {"GQNO", "NKC"}, {"GVNP", "RAI"}, {"DGAA", "ACC"}, {"DXXX", "LFW"}, {"DBBB", "COO"},
  {"DRRN", "NIM"}, {"DFFD", "OUA"}, {"FZAA", "FIH"}, {"FCBB", "BZV"}, {"FKKD", "DLA"}, {"FKYS", "NSI"},
  {"FOOL", "LBV"}, {"FTTJ", "NDJ"}, {"FEFF", "BGF"}, {"FPST", "TMS"}, {"FGSL", "SSG"}, {"HLLT", "TIP"},
  {"HLLM", "MJI"}, {"HLLB", "BEN"}, {"FBSK", "GBE"}, {"FVHA", "HRE"}, {"FVBU", "BUQ"}, {"FLKK", "LUN"},
  {"FLSK", "NLA"}, {"FWCL", "BLZ"}, {"FWKI", "LLW"}, {"FMMI", "TNR"}, {"FSIA", "SEZ"}, {"FQMA", "MPM"},
  {"FQBR", "BEW"}, {"FYWH", "WDH"},
  // South America
  {"MPTO", "PTY"}, {"SKBO", "BOG"}, {"SPJC", "LIM"}, {"SCEL", "SCL"}, {"SAEZ", "EZE"}, {"SBGR", "GRU"},
  {"SBGL", "GIG"}, {"SBSP", "CGH"}, {"SBKP", "VCP"}, {"SBRJ", "SDU"}, {"SLLP", "LPB"}, {"SLVR", "VVI"},
  {"SVMI", "CCS"}, {"SEQM", "UIO"}, {"SKSM", "SMR"},
  // Caribbean / Central America
  {"MUHA", "HAV"}, {"MDSD", "SDQ"}, {"MDPC", "PUJ"}, {"TJSJ", "SJU"}, {"MKJP", "KIN"}, {"MWCR", "GCM"},
  {"TTPP", "POS"}, {"TBPB", "BGI"}, {"TLPL", "UVF"}, {"MROC", "SJO"}, {"MGGT", "GUA"}, {"MHLM", "SAP"},
  {"MNMG", "MGA"}, {"MSLP", "SAL"}, {"MZBZ", "BZE"}, {"MYNN", "NAS"}, {"TAPA", "ANU"},
  // Europe
  {"EGLL", "LHR"}, {"LFPG", "CDG"}, {"EDDF", "FRA"}, {"EHAM", "AMS"}, {"UUEE", "SVO"}, {"LEMD", "MAD"},
  {"LIRF", "FCO"}, {"LPPT", "LIS"}, {"ESSA", "ARN"}, {"EKCH", "CPH"}, {"ENGM", "OSL"}, {"EFHK", "HEL"},
  {"LSZH", "ZRH"}, {"LOWW", "VIE"}, {"EBBR", "BRU"}, {"EPWA", "WAW"}, {"LKPR", "PRG"}, {"LDZA", "ZAG"},
  {"LGAV", "ATH"}, {"LTFM", "IST"}, {"EIDW", "DUB"}, {"ELLX", "LUX"}, {"LROP", "OTP"}, {"LBSF", "SOF"},
  {"LYBE", "BEG"}, {"LQSA", "SJJ"}, {"LWSK", "SKP"}, {"LATI", "TIA"}, {"LYPG", "TGD"}, {"LCLK", "LCA"},
  {"BIKF", "KEF"}, {"EVRA", "RIX"}, {"EYVI", "VNO"}, {"EETN", "TLL"}, {"UKBB", "KBP"}, {"UMMS", "MSQ"},
  {"LUKK", "RMO"}, {"BGSF", "SFJ"}, {"EKVG", "FAE"}, {"BGGH", "GOH"},
  // North America
  {"KATL", "ATL"}, {"KLAX", "LAX"}, {"KJFK", "JFK"}, {"KSFO", "SFO"}, {"KORD", "ORD"}, {"KDFW", "DFW"},
  {"KDEN", "DEN"}, {"KSEA", "SEA"}, {"KEWR", "EWR"}, {"KIAD", "IAD"}, {"KMIA", "MIA"}, {"KBOS", "BOS"},
  {"KLAS", "LAS"}, {"KPHX", "PHX"}, {"KIAH", "IAH"}, {"KCLT", "CLT"}, {"KMSP", "MSP"}, {"KDTW", "DTW"},
  {"KPHL", "PHL"}, {"KMCO", "MCO"}, {"PHNL", "HNL"}, {"PANC", "ANC"}, {"KSLC", "SLC"}, {"KSAN", "SAN"},
  {"KPDX", "PDX"}, {"KBWI", "BWI"}, {"KLGA", "LGA"}, {"KMDW", "MDW"}, {"KBNA", "BNA"}, {"KAUS", "AUS"},
  {"CYYZ", "YYZ"}, {"CYVR", "YVR"}, {"CYUL", "YUL"}, {"CYYC", "YYC"}, {"CYOW", "YOW"}, {"CYEG", "YEG"},
  {"CYWG", "YWG"}, {"CYHZ", "YHZ"}, {"MMMX", "MEX"}, {"MMUN", "CUN"}, {"MMGL", "GDL"}, {"MMMY", "MTY"},
  {"MMTJ", "TIJ"}, {"MMPR", "PVR"}, {"MMSD", "SJD"}, {"MMMD", "MID"},
  // Oceania
  {"YSSY", "SYD"}, {"YMML", "MEL"}, {"YBBN", "BNE"}, {"YPPH", "PER"}, {"YPAD", "ADL"}, {"NZAA", "AKL"},
  {"NFFN", "NAN"}, {"NVVV", "VLI"}, {"AYPY", "POM"}, {"AGGH", "HIR"}, {"NTAA", "PPT"}, {"NWWW", "NOU"},
  {"ANYN", "INU"}, {"NSFA", "APW"}, {"NCRG", "RAR"}, {"PGUM", "GUM"},
};
static const int AIRPORT_CODES_ICAO_IATA_COUNT = sizeof(AIRPORT_CODES_ICAO_IATA) / sizeof(AIRPORT_CODES_ICAO_IATA[0]);

// ---------------------------------------------------------------------------
// Web Server — Config Page
// ---------------------------------------------------------------------------
// #rrggbb <-> RGB565 conversion for the config page's color inputs.
static uint16_t hexToRgb565(const String& hex) {
  const char* s = hex.c_str();
  if (s[0] == '#') s++;
  long v = strtol(s, NULL, 16);
  uint8_t r = (v >> 16) & 0xFF, g = (v >> 8) & 0xFF, b = v & 0xFF;
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
static String rgb565ToHex(uint16_t c) {
  char buf[8];
  snprintf(buf, sizeof(buf), "#%02x%02x%02x",
           (c >> 8) & 0xF8, (c >> 3) & 0xFC, (c << 3) & 0xF8);
  return String(buf);
}

void initWebServer() {
  // GET / — serve config page
  server.on("/", []() {
    // Snapshot current values
    double hLat, hLon; float rMax; bool inv; uint8_t bright;
    bool showCS, showAir, showSpd, showFlt, showRte, showRg, showSq, showVr, showTy, showTr, preferTables;
    uint8_t maxB; uint16_t trailSamp, trailStale, cAlt, redrawMs, fetchSec;
    float swpSec, cNear; int16_t cVr;
    bool sApt, sKey, sComp, glowOn; uint8_t glowLen;
    bool ndOn; uint8_t ndStart, ndEnd, ndBright;
    uint16_t cSweep, cBlip, cBlipHi, cRings, cAirpt, cTrDep, cTrArr, cTrOver;
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    hLat = homeLat; hLon = homeLon; rMax = radarMaxKm; inv = invertColors; bright = brightness;
    showCS = showCallsign; showAir = showAirline; showSpd = showSpeed; showFlt = showFL; showRte = showRoute;
    showRg = showReg; showSq = showSquawk; showVr = showVRate; showTy = showType; showTr = showTraces; preferTables = preferLocalTables;
    maxB = maxBlipsShown; trailSamp = trailMaxSamples; trailStale = trailStaleSec;
    cNear = classNearKm; cAlt = classMaxAltFt; cVr = classVrateFpm;
    swpSec = sweepPeriodSec; redrawMs = radarRedrawMs; fetchSec = fetchIntervalSec;
    sApt = showAirports; sKey = showTrailKey; sComp = showCompass;
    glowOn = sweepGlow; glowLen = sweepGlowLen;
    ndOn = nightDimOn; ndStart = nightStartHr; ndEnd = nightEndHr; ndBright = nightBrightPct;
    cSweep = colSweep; cBlip = colBlip; cBlipHi = colBlipHi; cRings = colRings; cAirpt = colAirport;
    cTrDep = colTrailDep; cTrArr = colTrailArr; cTrOver = colTrailOver;
    if (configMutex) xSemaphoreGive(configMutex);

    String html = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CYD Plane Spotter</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
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
  input[type=range]{width:100%;padding:0;accent-color:#00ff88}
  input[type=color]{width:64px;height:34px;padding:2px;vertical-align:middle}
  select{padding:8px;border:1px solid #333;border-radius:6px;background:#0d0d1f;color:#fff;font-size:0.9em}
  .fr-row{display:flex;gap:6px;align-items:center;margin:8px 0}
  .fr-row input[type=text]{flex:1;min-width:0}
  .fr-row input[type=checkbox]{width:auto}
  .fr-row input[type=color]{width:44px;flex-shrink:0}
  .color-row{display:flex;align-items:center;justify-content:space-between;margin:8px 0}
  .color-row label{margin:0;color:#e0e0e0;font-size:0.95em}
  .note{color:#666;font-size:0.75em;margin:2px 0 6px}
</style></head><body>
<h1>&#9992; CYD Plane Spotter</h1>
<div class="card">
  <form id="cfg" onsubmit="save(event)">
    <div class="section-hd" style="border-top:none;padding-top:0;margin-top:0">WiFi Networks</div>
    <div class="note">Up to 4 saved networks, tried in order. If none are reachable the device starts its own "PlaneSpotter-Setup" AP &mdash; connect to it and open 192.168.4.1 to fix settings.</div>
    <label>Network 1 SSID / Password</label>
    <input type="text" name="wssid0" id="wssid0" maxlength="32" value=")rawliteral" + String(wifiNets[0].ssid) + R"rawliteral(">
    <input type="text" name="wpass0" id="wpass0" maxlength="64" value=")rawliteral" + String(wifiNets[0].pass) + R"rawliteral(">
    <label>Network 2 SSID / Password</label>
    <input type="text" name="wssid1" id="wssid1" maxlength="32" value=")rawliteral" + String(wifiNets[1].ssid) + R"rawliteral(">
    <input type="text" name="wpass1" id="wpass1" maxlength="64" value=")rawliteral" + String(wifiNets[1].pass) + R"rawliteral(">
    <label>Network 3 SSID / Password</label>
    <input type="text" name="wssid2" id="wssid2" maxlength="32" value=")rawliteral" + String(wifiNets[2].ssid) + R"rawliteral(">
    <input type="text" name="wpass2" id="wpass2" maxlength="64" value=")rawliteral" + String(wifiNets[2].pass) + R"rawliteral(">
    <label>Network 4 SSID / Password</label>
    <input type="text" name="wssid3" id="wssid3" maxlength="32" value=")rawliteral" + String(wifiNets[3].ssid) + R"rawliteral(">
    <input type="text" name="wpass3" id="wpass3" maxlength="64" value=")rawliteral" + String(wifiNets[3].pass) + R"rawliteral(">
    <div class="section-hd">Location &amp; Radar</div>
    <label>Home Latitude (&deg;)</label>
    <input type="number" step="any" name="lat" id="lat" value=")rawliteral" + String(hLat, 6) + R"rawliteral(" required>
    <label>Home Longitude (&deg;)</label>
    <input type="number" step="any" name="lon" id="lon" value=")rawliteral" + String(hLon, 6) + R"rawliteral(" required>
    <div id="map" style="height:300px;border-radius:8px;margin:8px 0;background:#0d0d1f"></div>
    <div class="note">Click the map to set home (needs internet in THIS browser for the map tiles). The green circle previews your radar range. Number fields stay in sync both ways.</div>
    <button type="button" onclick="useMyLocation()" style="margin-top:0;background:#2a2a4a;color:#e0e0e0">Use my current location</button>
    <div class="note" style="margin-bottom:8px">(browser may block location on plain http &mdash; the map click always works)</div>
    <label>Radar Max Range (km) &mdash; also sets data fetch radius</label>
    <input type="number" step="any" name="range" id="range" value=")rawliteral" + String(rMax, 1) + R"rawliteral(" required>
    <div class="switch-row">
      <label for="dark">Dark Mode</label>
      <label class="switch">
        <input type="checkbox" id="dark")rawliteral" + String(inv ? "" : " checked") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <label for="bright">Brightness &mdash; <span id="brightVal">)rawliteral" + String(bright) + R"rawliteral(</span>%</label>
    <input type="range" name="bright" id="bright" min="1" max="100" value=")rawliteral" + String(bright) + R"rawliteral(" oninput="document.getElementById('brightVal').textContent=this.value">
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
    <div class="switch-row">
      <label for="traces">Radar Flight Path Traces</label>
      <label class="switch">
        <input type="checkbox" id="traces")rawliteral" + String(showTr ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="switch-row">
      <label for="tables">Prefer Local Tables (fewer API calls)</label>
      <label class="switch">
        <input type="checkbox" id="tables")rawliteral" + String(preferTables ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="section-hd">Radar Display</div>
    <div class="note">Currently applied by the LCD-7B v2 display engine only.</div>
    <label>Max planes shown at once (1-20)</label>
    <input type="number" name="maxblips" id="maxblips" min="1" max="20" value=")rawliteral" + String(maxB) + R"rawliteral(">
    <label>Sweep revolution period (seconds)</label>
    <input type="number" step="0.1" name="swpsec" id="swpsec" min="1" max="60" value=")rawliteral" + String(swpSec, 1) + R"rawliteral(">
    <div class="switch-row">
      <label for="swpglow">Phosphor afterglow behind sweep</label>
      <label class="switch">
        <input type="checkbox" id="swpglow")rawliteral" + String(glowOn ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <label>Afterglow length (segments, 1-12)</label>
    <input type="number" name="swpglowlen" id="swpglowlen" min="1" max="12" value=")rawliteral" + String(glowLen) + R"rawliteral(">
    <label>Full screen refresh interval (ms)</label>
    <input type="number" step="50" name="redrawms" id="redrawms" min="200" max="5000" value=")rawliteral" + String(redrawMs) + R"rawliteral(">
    <div class="color-row"><label>Sweep line</label><input type="color" name="c_sweep" id="c_sweep" value=")rawliteral" + rgb565ToHex(cSweep) + R"rawliteral("></div>
    <div class="color-row"><label>Plane (after sweep passes)</label><input type="color" name="c_blip" id="c_blip" value=")rawliteral" + rgb565ToHex(cBlip) + R"rawliteral("></div>
    <div class="color-row"><label>Plane (just behind sweep)</label><input type="color" name="c_bliphi" id="c_bliphi" value=")rawliteral" + rgb565ToHex(cBlipHi) + R"rawliteral("></div>
    <div class="color-row"><label>Range rings &amp; axes</label><input type="color" name="c_rings" id="c_rings" value=")rawliteral" + rgb565ToHex(cRings) + R"rawliteral("></div>
    <div class="color-row"><label>Airports</label><input type="color" name="c_airpt" id="c_airpt" value=")rawliteral" + rgb565ToHex(cAirpt) + R"rawliteral("></div>
    <div class="switch-row">
      <label for="showapt">Show airports on radar</label>
      <label class="switch">
        <input type="checkbox" id="showapt")rawliteral" + String(sApt ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="switch-row">
      <label for="showcomp">Show compass letters (N/E/S/W)</label>
      <label class="switch">
        <input type="checkbox" id="showcomp")rawliteral" + String(sComp ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="switch-row">
      <label for="showkey">Show trace color key</label>
      <label class="switch">
        <input type="checkbox" id="showkey")rawliteral" + String(sKey ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <div class="section-hd">Flight Path Traces</div>
    <div class="color-row"><label>Taking off</label><input type="color" name="c_trdep" id="c_trdep" value=")rawliteral" + rgb565ToHex(cTrDep) + R"rawliteral("></div>
    <div class="color-row"><label>Landing</label><input type="color" name="c_trarr" id="c_trarr" value=")rawliteral" + rgb565ToHex(cTrArr) + R"rawliteral("></div>
    <div class="color-row"><label>Flying over</label><input type="color" name="c_trover" id="c_trover" value=")rawliteral" + rgb565ToHex(cTrOver) + R"rawliteral("></div>
    <label>Max trail length (samples, 50-1800)</label>
    <input type="number" step="50" name="trailsamp" id="trailsamp" min="50" max="1800" value=")rawliteral" + String(trailSamp) + R"rawliteral(">
    <label>Drop trail after seconds unseen (5-300)</label>
    <input type="number" name="trailstale" id="trailstale" min="5" max="300" value=")rawliteral" + String(trailStale) + R"rawliteral(">
    <div class="section-hd">Takeoff / Landing Classification</div>
    <label>Near-airport distance (km)</label>
    <input type="number" step="0.5" name="cnearkm" id="cnearkm" min="1" max="200" value=")rawliteral" + String(cNear, 1) + R"rawliteral(">
    <label>Max altitude (ft)</label>
    <input type="number" step="500" name="cmaxalt" id="cmaxalt" min="500" max="45000" value=")rawliteral" + String(cAlt) + R"rawliteral(">
    <label>Min vertical rate (fpm)</label>
    <input type="number" step="50" name="cvrate" id="cvrate" min="50" max="3000" value=")rawliteral" + String(cVr) + R"rawliteral(">
    <div class="section-hd">Day / Night Cycle</div>
    <div class="switch-row">
      <label for="nd_on">Auto-dim backlight at night</label>
      <label class="switch">
        <input type="checkbox" id="nd_on")rawliteral" + String(ndOn ? " checked" : "") + R"rawliteral(>
        <span class="slider"></span>
      </label>
    </div>
    <label>Night starts at hour (0-23)</label>
    <input type="number" name="nd_start" id="nd_start" min="0" max="23" value=")rawliteral" + String(ndStart) + R"rawliteral(">
    <label>Night ends at hour (0-23)</label>
    <input type="number" name="nd_end" id="nd_end" min="0" max="23" value=")rawliteral" + String(ndEnd) + R"rawliteral(">
    <label for="nd_bright">Night brightness &mdash; <span id="ndBrightVal">)rawliteral" + String(ndBright) + R"rawliteral(</span>%</label>
    <input type="range" name="nd_bright" id="nd_bright" min="1" max="100" value=")rawliteral" + String(ndBright) + R"rawliteral(" oninput="document.getElementById('ndBrightVal').textContent=this.value">
    <div class="section-hd">Data Fetch</div>
    <label>Aircraft poll interval (seconds, 5-600)</label>
    <input type="number" name="fetchsec" id="fetchsec" min="5" max="600" value=")rawliteral" + String(fetchSec) + R"rawliteral(">
    <div class="section-hd">Traffic Filter / Watchlist</div>
    <div class="note">Case-insensitive prefix match. <b>Highlight</b> colors blip+trail; <b>Hide</b> removes it; <b>Only</b> shows exclusively matching planes (when any Only rule is on); <b>Alert</b> adds a banner when a match is in range. Quiet mode dims ALL traffic while nothing watched is airborne.</div>
    <div class="fr-row">
      <input type="checkbox" id="fr_en0">
      <select id="fr_m0"><option value="0">Callsign</option><option value="1">Reg</option><option value="2">Type</option></select>
      <input type="text" id="fr_t0" maxlength="11" placeholder="e.g. KAL">
      <select id="fr_a0"><option value="0">Highlight</option><option value="1">Hide</option><option value="2">Only</option><option value="3">Alert</option></select>
      <input type="color" id="fr_c0">
    </div>
    <div class="fr-row">
      <input type="checkbox" id="fr_en1">
      <select id="fr_m1"><option value="0">Callsign</option><option value="1">Reg</option><option value="2">Type</option></select>
      <input type="text" id="fr_t1" maxlength="11" placeholder="">
      <select id="fr_a1"><option value="0">Highlight</option><option value="1">Hide</option><option value="2">Only</option><option value="3">Alert</option></select>
      <input type="color" id="fr_c1">
    </div>
    <div class="fr-row">
      <input type="checkbox" id="fr_en2">
      <select id="fr_m2"><option value="0">Callsign</option><option value="1">Reg</option><option value="2">Type</option></select>
      <input type="text" id="fr_t2" maxlength="11" placeholder="">
      <select id="fr_a2"><option value="0">Highlight</option><option value="1">Hide</option><option value="2">Only</option><option value="3">Alert</option></select>
      <input type="color" id="fr_c2">
    </div>
    <div class="fr-row">
      <input type="checkbox" id="fr_en3">
      <select id="fr_m3"><option value="0">Callsign</option><option value="1">Reg</option><option value="2">Type</option></select>
      <input type="text" id="fr_t3" maxlength="11" placeholder="">
      <select id="fr_a3"><option value="0">Highlight</option><option value="1">Hide</option><option value="2">Only</option><option value="3">Alert</option></select>
      <input type="color" id="fr_c3">
    </div>
    <div class="fr-row">
      <input type="checkbox" id="fr_en4">
      <select id="fr_m4"><option value="0">Callsign</option><option value="1">Reg</option><option value="2">Type</option></select>
      <input type="text" id="fr_t4" maxlength="11" placeholder="">
      <select id="fr_a4"><option value="0">Highlight</option><option value="1">Hide</option><option value="2">Only</option><option value="3">Alert</option></select>
      <input type="color" id="fr_c4">
    </div>
    <div class="switch-row">
      <label for="fr_quiet">Quiet mode (dim all traffic unless watched is up)</label>
      <label class="switch">
        <input type="checkbox" id="fr_quiet">
        <span class="slider"></span>
      </label>
    </div>
    <button type="submit">Save Settings</button>
  </form>
  <div class="saved" id="msg">&#10003; Settings saved!</div>
</div>
<div class="card">
  <h2 style="color:#00ff88;font-size:1em;margin:0 0 8px">Firmware Update</h2>
  <form id="ota" onsubmit="return doOta(event)">
    <input type="file" id="fw" accept=".bin" required style="width:100%;box-sizing:border-box;margin:6px 0;color:#e0e0e0">
    <button type="submit">Upload &amp; Flash</button>
  </form>
  <div id="otaMsg" style="text-align:center;margin-top:8px;font-size:0.85em;color:#aaa"></div>
</div>
<div class="info">IP: )rawliteral" + WiFi.localIP().toString() + R"rawliteral(</div>
<script>
// --- Location map (Leaflet + OSM; tiles load in the browser, not the device)
var map = L.map('map').setView([)rawliteral" + String(hLat, 6) + R"rawliteral(, )rawliteral" + String(hLon, 6) + R"rawliteral(], 9);
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {maxZoom: 19, attribution: '&copy; OpenStreetMap contributors'}).addTo(map);
var marker = L.marker([)rawliteral" + String(hLat, 6) + R"rawliteral(, )rawliteral" + String(hLon, 6) + R"rawliteral(], {draggable: true}).addTo(map);
var rangeCircle = L.circle([)rawliteral" + String(hLat, 6) + R"rawliteral(, )rawliteral" + String(hLon, 6) + R"rawliteral(], {radius: )rawliteral" + String(rMax * 1000.0f, 0) + R"rawliteral(, color: '#00ff88', weight: 1, fillOpacity: 0.05}).addTo(map);
function setLatLon(lat, lon) {
  document.getElementById('lat').value = lat.toFixed(6);
  document.getElementById('lon').value = lon.toFixed(6);
  marker.setLatLng([lat, lon]);
  rangeCircle.setLatLng([lat, lon]);
}
map.on('click', function(e) { setLatLon(e.latlng.lat, e.latlng.lng); });
marker.on('dragend', function() { var p = marker.getLatLng(); setLatLon(p.lat, p.lng); });
function syncFromInputs() {
  var la = parseFloat(document.getElementById('lat').value);
  var lo = parseFloat(document.getElementById('lon').value);
  if (!isNaN(la) && !isNaN(lo)) { marker.setLatLng([la, lo]); rangeCircle.setLatLng([la, lo]); map.panTo([la, lo]); }
}
document.getElementById('lat').addEventListener('change', syncFromInputs);
document.getElementById('lon').addEventListener('change', syncFromInputs);
document.getElementById('range').addEventListener('change', function() {
  var r = parseFloat(document.getElementById('range').value);
  if (!isNaN(r)) rangeCircle.setRadius(r * 1000);
});
function useMyLocation() {
  if (!navigator.geolocation) return;
  navigator.geolocation.getCurrentPosition(function(pos) {
    setLatLon(pos.coords.latitude, pos.coords.longitude);
    map.setView([pos.coords.latitude, pos.coords.longitude], 11);
  });
}
// --- filter rules init
function initRule(i,en,m,t,a,c){
  document.getElementById('fr_en'+i).checked=(en==1);
  document.getElementById('fr_m'+i).value=String(m);
  document.getElementById('fr_t'+i).value=t;
  document.getElementById('fr_a'+i).value=String(a);
  document.getElementById('fr_c'+i).value=c;
}
initRule(0,)rawliteral" + String(filterRules[0].enabled ? 1 : 0) + "," + String(filterRules[0].match) + ",'" + String(filterRules[0].text) + "'," + String(filterRules[0].action) + ",'" + rgb565ToHex(filterRules[0].color) + R"rawliteral(');
initRule(1,)rawliteral" + String(filterRules[1].enabled ? 1 : 0) + "," + String(filterRules[1].match) + ",'" + String(filterRules[1].text) + "'," + String(filterRules[1].action) + ",'" + rgb565ToHex(filterRules[1].color) + R"rawliteral(');
initRule(2,)rawliteral" + String(filterRules[2].enabled ? 1 : 0) + "," + String(filterRules[2].match) + ",'" + String(filterRules[2].text) + "'," + String(filterRules[2].action) + ",'" + rgb565ToHex(filterRules[2].color) + R"rawliteral(');
initRule(3,)rawliteral" + String(filterRules[3].enabled ? 1 : 0) + "," + String(filterRules[3].match) + ",'" + String(filterRules[3].text) + "'," + String(filterRules[3].action) + ",'" + rgb565ToHex(filterRules[3].color) + R"rawliteral(');
initRule(4,)rawliteral" + String(filterRules[4].enabled ? 1 : 0) + "," + String(filterRules[4].match) + ",'" + String(filterRules[4].text) + "'," + String(filterRules[4].action) + ",'" + rgb565ToHex(filterRules[4].color) + R"rawliteral(');
document.getElementById('fr_quiet').checked = )rawliteral" + String(filterQuiet ? "true" : "false") + R"rawliteral(;
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
         '&wssid0='+encodeURIComponent(document.getElementById('wssid0').value)+
         '&wpass0='+encodeURIComponent(document.getElementById('wpass0').value)+
         '&wssid1='+encodeURIComponent(document.getElementById('wssid1').value)+
         '&wpass1='+encodeURIComponent(document.getElementById('wpass1').value)+
         '&wssid2='+encodeURIComponent(document.getElementById('wssid2').value)+
         '&wpass2='+encodeURIComponent(document.getElementById('wpass2').value)+
         '&wssid3='+encodeURIComponent(document.getElementById('wssid3').value)+
         '&wpass3='+encodeURIComponent(document.getElementById('wpass3').value)+
         '&range='+encodeURIComponent(document.getElementById('range').value)+
         '&dark='+(document.getElementById('dark').checked?'1':'0')+
         '&bright='+encodeURIComponent(document.getElementById('bright').value)+
         '&callsign='+(document.getElementById('callsign').checked?'1':'0')+
         '&airline='+(document.getElementById('airline').checked?'1':'0')+
         '&speed='+(document.getElementById('speed').checked?'1':'0')+
         '&fl='+(document.getElementById('fl').checked?'1':'0')+
         '&route='+(document.getElementById('route').checked?'1':'0')+
         '&reg='+(document.getElementById('reg').checked?'1':'0')+
         '&squawk='+(document.getElementById('squawk').checked?'1':'0')+
         '&vrate='+(document.getElementById('vrate').checked?'1':'0')+
         '&type='+(document.getElementById('type').checked?'1':'0')+
         '&traces='+(document.getElementById('traces').checked?'1':'0')+
         '&tables='+(document.getElementById('tables').checked?'1':'0')+
         '&maxblips='+encodeURIComponent(document.getElementById('maxblips').value)+
         '&swpsec='+encodeURIComponent(document.getElementById('swpsec').value)+
         '&redrawms='+encodeURIComponent(document.getElementById('redrawms').value)+
         '&fetchsec='+encodeURIComponent(document.getElementById('fetchsec').value)+
         '&trailsamp='+encodeURIComponent(document.getElementById('trailsamp').value)+
         '&trailstale='+encodeURIComponent(document.getElementById('trailstale').value)+
         '&cnearkm='+encodeURIComponent(document.getElementById('cnearkm').value)+
         '&cmaxalt='+encodeURIComponent(document.getElementById('cmaxalt').value)+
         '&cvrate='+encodeURIComponent(document.getElementById('cvrate').value)+
         '&c_sweep='+encodeURIComponent(document.getElementById('c_sweep').value)+
         '&c_blip='+encodeURIComponent(document.getElementById('c_blip').value)+
         '&c_bliphi='+encodeURIComponent(document.getElementById('c_bliphi').value)+
         '&c_rings='+encodeURIComponent(document.getElementById('c_rings').value)+
         '&c_airpt='+encodeURIComponent(document.getElementById('c_airpt').value)+
         '&c_trdep='+encodeURIComponent(document.getElementById('c_trdep').value)+
         '&c_trarr='+encodeURIComponent(document.getElementById('c_trarr').value)+
         '&c_trover='+encodeURIComponent(document.getElementById('c_trover').value)+
         '&showapt='+(document.getElementById('showapt').checked?'1':'0')+
         '&showcomp='+(document.getElementById('showcomp').checked?'1':'0')+
         '&showkey='+(document.getElementById('showkey').checked?'1':'0')+
         '&swpglow='+(document.getElementById('swpglow').checked?'1':'0')+
         '&swpglowlen='+encodeURIComponent(document.getElementById('swpglowlen').value)+
         '&nd_on='+(document.getElementById('nd_on').checked?'1':'0')+
         '&nd_start='+encodeURIComponent(document.getElementById('nd_start').value)+
         '&nd_end='+encodeURIComponent(document.getElementById('nd_end').value)+
         '&nd_bright='+encodeURIComponent(document.getElementById('nd_bright').value)+
         '&fr_quiet='+(document.getElementById('fr_quiet').checked?'1':'0')+
         ruleArgs(0)+ruleArgs(1)+ruleArgs(2)+ruleArgs(3)+ruleArgs(4));
}
function ruleArgs(i){
  return '&fr_en'+i+'='+(document.getElementById('fr_en'+i).checked?'1':'0')+
         '&fr_m'+i+'='+document.getElementById('fr_m'+i).value+
         '&fr_t'+i+'='+encodeURIComponent(document.getElementById('fr_t'+i).value)+
         '&fr_a'+i+'='+document.getElementById('fr_a'+i).value+
         '&fr_c'+i+'='+encodeURIComponent(document.getElementById('fr_c'+i).value);
}
function doOta(e){
  e.preventDefault();
  var f=document.getElementById('fw').files[0];
  if(!f) return false;
  if(!confirm('Upload '+f.name+' and reboot the device? Make sure this .bin was built for this exact board.')) return false;
  var fd=new FormData();
  fd.append('update', f);
  var x=new XMLHttpRequest();
  x.open('POST','/update',true);
  document.getElementById('otaMsg').textContent='Uploading...';
  x.onload=function(){
    document.getElementById('otaMsg').textContent = x.status==200 ? x.responseText : ('Upload failed: '+x.responseText);
  };
  x.onerror=function(){
    document.getElementById('otaMsg').textContent='Upload error';
  };
  x.send(fd);
  return false;
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
    uint8_t newBrightness = server.hasArg("bright") ? (uint8_t)server.arg("bright").toInt() : brightness;
    bool   newShowCallsign = server.hasArg("callsign") ? (server.arg("callsign") == "1") : showCallsign;
    bool   newShowAirline = server.hasArg("airline") ? (server.arg("airline") == "1") : showAirline;
    bool   newShowSpeed = server.hasArg("speed") ? (server.arg("speed") == "1") : showSpeed;
    bool   newShowFL = server.hasArg("fl") ? (server.arg("fl") == "1") : showFL;
    bool   newShowRoute = server.hasArg("route") ? (server.arg("route") == "1") : showRoute;
    bool   newShowReg = server.hasArg("reg") ? (server.arg("reg") == "1") : showReg;
    bool   newShowSquawk = server.hasArg("squawk") ? (server.arg("squawk") == "1") : showSquawk;
    bool   newShowVRate = server.hasArg("vrate") ? (server.arg("vrate") == "1") : showVRate;
    bool   newShowType = server.hasArg("type") ? (server.arg("type") == "1") : showType;
    bool   newShowTraces = server.hasArg("traces") ? (server.arg("traces") == "1") : showTraces;
    bool   newPreferTables = server.hasArg("tables") ? (server.arg("tables") == "1") : preferLocalTables;

    // Extended radar/display settings (all optional — absent args keep current)
    uint8_t  newMaxBlips = server.hasArg("maxblips") ? (uint8_t)server.arg("maxblips").toInt() : maxBlipsShown;
    float    newSwpSec = server.hasArg("swpsec") ? server.arg("swpsec").toFloat() : sweepPeriodSec;
    uint16_t newRedrawMs = server.hasArg("redrawms") ? (uint16_t)server.arg("redrawms").toInt() : radarRedrawMs;
    uint16_t newFetchSec = server.hasArg("fetchsec") ? (uint16_t)server.arg("fetchsec").toInt() : fetchIntervalSec;
    uint16_t newTrailSamp = server.hasArg("trailsamp") ? (uint16_t)server.arg("trailsamp").toInt() : trailMaxSamples;
    uint16_t newTrailStale = server.hasArg("trailstale") ? (uint16_t)server.arg("trailstale").toInt() : trailStaleSec;
    float    newCNear = server.hasArg("cnearkm") ? server.arg("cnearkm").toFloat() : classNearKm;
    uint16_t newCAlt = server.hasArg("cmaxalt") ? (uint16_t)server.arg("cmaxalt").toInt() : classMaxAltFt;
    int16_t  newCVr = server.hasArg("cvrate") ? (int16_t)server.arg("cvrate").toInt() : classVrateFpm;
    bool     newShowApt = server.hasArg("showapt") ? (server.arg("showapt") == "1") : showAirports;
    bool     newShowKey = server.hasArg("showkey") ? (server.arg("showkey") == "1") : showTrailKey;
    bool     newShowComp = server.hasArg("showcomp") ? (server.arg("showcomp") == "1") : showCompass;
    uint16_t newColSweep = server.hasArg("c_sweep") ? hexToRgb565(server.arg("c_sweep")) : colSweep;
    uint16_t newColBlip = server.hasArg("c_blip") ? hexToRgb565(server.arg("c_blip")) : colBlip;
    uint16_t newColBlipHi = server.hasArg("c_bliphi") ? hexToRgb565(server.arg("c_bliphi")) : colBlipHi;
    uint16_t newColRings = server.hasArg("c_rings") ? hexToRgb565(server.arg("c_rings")) : colRings;
    uint16_t newColAirpt = server.hasArg("c_airpt") ? hexToRgb565(server.arg("c_airpt")) : colAirport;
    uint16_t newColTrDep = server.hasArg("c_trdep") ? hexToRgb565(server.arg("c_trdep")) : colTrailDep;
    uint16_t newColTrArr = server.hasArg("c_trarr") ? hexToRgb565(server.arg("c_trarr")) : colTrailArr;
    uint16_t newColTrOver = server.hasArg("c_trover") ? hexToRgb565(server.arg("c_trover")) : colTrailOver;

    // Traffic filter rules (optional args keep current rule state)
    FilterRule newRules[FILTER_MAX_RULES];
    memcpy(newRules, filterRules, sizeof(newRules));
    for (uint8_t i = 0; i < FILTER_MAX_RULES; i++) {
      char ken[9], km[8], kt[8], ka[8], kc[8];
      snprintf(ken, sizeof(ken), "fr_en%d", i);
      snprintf(km, sizeof(km), "fr_m%d", i);
      snprintf(kt, sizeof(kt), "fr_t%d", i);
      snprintf(ka, sizeof(ka), "fr_a%d", i);
      snprintf(kc, sizeof(kc), "fr_c%d", i);
      if (server.hasArg(ken)) {
        newRules[i].enabled = server.arg(ken) == "1";
        newRules[i].match = (uint8_t)constrain(server.arg(km).toInt(), 0, 2);
        strncpy(newRules[i].text, server.arg(kt).c_str(), 11); newRules[i].text[11] = '\0';
        newRules[i].action = (uint8_t)constrain(server.arg(ka).toInt(), 0, 3);
        newRules[i].color = hexToRgb565(server.arg(kc));
      }
    }
    bool newQuiet = server.hasArg("fr_quiet") ? (server.arg("fr_quiet") == "1") : filterQuiet;
    bool newGlowOn = server.hasArg("swpglow") ? (server.arg("swpglow") == "1") : sweepGlow;
    uint8_t newGlowLen = server.hasArg("swpglowlen") ? (uint8_t)server.arg("swpglowlen").toInt() : sweepGlowLen;
    if (newGlowLen < 1) newGlowLen = 1;
    if (newGlowLen > 12) newGlowLen = 12;

    // WiFi network slots (optional). strncpy under configMutex since
    // wifiMaintain() reads the slots on Core 1; NVS writes stay outside.
    // If anything changed, drop the current connection — wifiMaintain()
    // reconnects within seconds.
    bool wifiChanged = false;
    for (uint8_t i = 0; i < WIFI_MAX_NETWORKS; i++) {
      char ks[10], kp[10];
      snprintf(ks, sizeof(ks), "wssid%d", i);
      snprintf(kp, sizeof(kp), "wpass%d", i);
      if (server.hasArg(ks)) {
        String s = server.arg(ks);
        String p = server.hasArg(kp) ? server.arg(kp) : "";
        if (s != String(wifiNets[i].ssid) || p != String(wifiNets[i].pass)) wifiChanged = true;
        if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
        strncpy(wifiNets[i].ssid, s.c_str(), 32); wifiNets[i].ssid[32] = '\0';
        strncpy(wifiNets[i].pass, p.c_str(), 64); wifiNets[i].pass[64] = '\0';
        if (configMutex) xSemaphoreGive(configMutex);
        prefs.putString(ks, s);
        prefs.putString(kp, p);
      }
    }

    // Basic validation
    if (newLat < -90 || newLat > 90 || newLon < -180 || newLon > 180 ||
        newRng <= 0 || newRng > 500) {
      server.send(400, "text/plain", "Invalid values");
      return;
    }
    if (newBrightness < 1) newBrightness = 1;
    if (newBrightness > 100) newBrightness = 100;
    // Clamp the extended settings to sane ranges
    if (newMaxBlips < 1) newMaxBlips = 1;
    if (newMaxBlips > MAX_BLIPS) newMaxBlips = MAX_BLIPS;
    if (newSwpSec < 1.0f) newSwpSec = 1.0f;
    if (newSwpSec > 60.0f) newSwpSec = 60.0f;
    if (newRedrawMs < 200) newRedrawMs = 200;
    if (newRedrawMs > 5000) newRedrawMs = 5000;
    if (newFetchSec < 5) newFetchSec = 5;
    if (newFetchSec > 600) newFetchSec = 600;
    if (newTrailSamp < 50) newTrailSamp = 50;
    if (newTrailSamp > 1800) newTrailSamp = 1800;
    if (newTrailStale < 5) newTrailStale = 5;
    if (newTrailStale > 300) newTrailStale = 300;
    if (newCNear < 1.0f) newCNear = 1.0f;
    if (newCNear > 200.0f) newCNear = 200.0f;
    if (newCAlt < 500) newCAlt = 500;
    if (newCAlt > 45000) newCAlt = 45000;
    if (newCVr < 50) newCVr = 50;
    if (newCVr > 3000) newCVr = 3000;

    // Atomically update
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    homeLat = newLat;
    homeLon = newLon;
    radarMaxKm = newRng;
    invertColors = newInvert;
    brightness = newBrightness;
    showCallsign = newShowCallsign;
    showAirline = newShowAirline;
    showSpeed = newShowSpeed;
    showFL = newShowFL;
    showRoute = newShowRoute;
    showReg = newShowReg;
    showSquawk = newShowSquawk;
    showVRate = newShowVRate;
    showType = newShowType;
    showTraces = newShowTraces;
    preferLocalTables = newPreferTables;
    maxBlipsShown = newMaxBlips;
    sweepPeriodSec = newSwpSec;
    radarRedrawMs = newRedrawMs;
    fetchIntervalSec = newFetchSec;
    trailMaxSamples = newTrailSamp;
    trailStaleSec = newTrailStale;
    classNearKm = newCNear;
    classMaxAltFt = newCAlt;
    classVrateFpm = newCVr;
    showAirports = newShowApt;
    showTrailKey = newShowKey;
    showCompass = newShowComp;
    colSweep = newColSweep;
    colBlip = newColBlip;
    colBlipHi = newColBlipHi;
    colRings = newColRings;
    colAirport = newColAirpt;
    colTrailDep = newColTrDep;
    colTrailArr = newColTrArr;
    colTrailOver = newColTrOver;
    memcpy(filterRules, newRules, sizeof(filterRules));
    filterQuiet = newQuiet;
    sweepGlow = newGlowOn;
    sweepGlowLen = newGlowLen;
    if (configMutex) xSemaphoreGive(configMutex);
    applyInvertColors(newInvert);
    applyBrightness(newBrightness);

    // Persist to NVS
    prefs.putDouble("lat", newLat);
    prefs.putDouble("lon", newLon);
    prefs.putFloat("range", newRng);
    prefs.putBool("invert", newInvert);
    prefs.putUChar("bright", newBrightness);
    prefs.putBool("callsign", newShowCallsign);
    prefs.putBool("airline", newShowAirline);
    prefs.putBool("speed", newShowSpeed);
    prefs.putBool("fl", newShowFL);
    prefs.putBool("route", newShowRoute);
    prefs.putBool("reg", newShowReg);
    prefs.putBool("squawk", newShowSquawk);
    prefs.putBool("vrate", newShowVRate);
    prefs.putBool("type", newShowType);
    prefs.putBool("traces", newShowTraces);
    prefs.putBool("tables", newPreferTables);
    prefs.putUChar("maxblips", newMaxBlips);
    prefs.putFloat("swpsec", newSwpSec);
    prefs.putUShort("redrawms", newRedrawMs);
    prefs.putUShort("fetchsec", newFetchSec);
    prefs.putUShort("trailsamp", newTrailSamp);
    prefs.putUShort("trailstale", newTrailStale);
    prefs.putFloat("cnearkm", newCNear);
    prefs.putUShort("cmaxalt", newCAlt);
    prefs.putShort("cvrate", newCVr);
    prefs.putBool("showapt", newShowApt);
    prefs.putBool("showkey", newShowKey);
    prefs.putBool("showcomp", newShowComp);
    prefs.putUShort("c_sweep", newColSweep);
    prefs.putUShort("c_blip", newColBlip);
    prefs.putUShort("c_bliphi", newColBlipHi);
    prefs.putUShort("c_rings", newColRings);
    prefs.putUShort("c_airpt", newColAirpt);
    prefs.putUShort("c_trdep", newColTrDep);
    prefs.putUShort("c_trarr", newColTrArr);
    prefs.putUShort("c_trover", newColTrOver);
    for (uint8_t i = 0; i < FILTER_MAX_RULES; i++) {
      char ken[9], km[8], kt[8], ka[8], kc[8];
      snprintf(ken, sizeof(ken), "fr_en%d", i);
      snprintf(km, sizeof(km), "fr_m%d", i);
      snprintf(kt, sizeof(kt), "fr_t%d", i);
      snprintf(ka, sizeof(ka), "fr_a%d", i);
      snprintf(kc, sizeof(kc), "fr_c%d", i);
      prefs.putBool(ken, newRules[i].enabled);
      prefs.putUChar(km, newRules[i].match);
      prefs.putString(kt, newRules[i].text);
      prefs.putUChar(ka, newRules[i].action);
      prefs.putUShort(kc, newRules[i].color);
    }
    prefs.putBool("fr_quiet", newQuiet);
    prefs.putBool("swpglow", newGlowOn);
    prefs.putUChar("swpglowlen", newGlowLen);

    Serial.printf("Config saved: lat=%.6f lon=%.6f range=%.1f invert=%d callsign=%d airline=%d speed=%d fl=%d route=%d reg=%d squawk=%d vrate=%d type=%d traces=%d tables=%d\n",
                  newLat, newLon, newRng, newInvert, newShowCallsign, newShowAirline, newShowSpeed, newShowFL,
                  newShowRoute, newShowReg, newShowSquawk, newShowVRate, newShowType, newShowTraces, newPreferTables);
    if (wifiChanged) {
      Serial.println("[wifi] credentials changed via web UI — reconnecting");
      WiFi.disconnect(false);  // wifiMaintain() picks it up within seconds
    }
    server.send(200, "text/plain", "OK");
  });

  // POST /update — OTA firmware upload (multipart file upload). This flashes
  // whatever .bin is uploaded into the inactive OTA app slot and reboots into
  // it — there's no board-identity check, since the firmware itself has no
  // reliable way to know what board a .bin was built for before flashing it.
  // The web UI confirm() dialog is the only guard.
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    if (Update.hasError()) {
      server.send(200, "text/plain", "Update failed, still running old firmware");
    } else {
      server.send(200, "text/plain", "Update OK, rebooting...");
    }
    delay(500);
    ESP.restart();
  }, []() {
    HTTPUpload &upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("[ota] Update start: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("[ota] Update success: %u bytes\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  server.begin();
  Serial.println("Web server started on http://" + WiFi.localIP().toString() + "/");
}

// Serves all HTTP requests from Core 0 (started in main.cpp's setup).
// Core 1 (rendering/touch) never calls handleClient(), so config-page
// visits, /health polls and OTA uploads no longer steal time from the
// render path. WebServer is single-threaded per instance but not pinned
// to a core; all routes are registered in initWebServer() before this
// task starts, and handlers only touch shared state under configMutex.
void webServerTask(void* param) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
