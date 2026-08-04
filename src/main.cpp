/*
 * CYD28 Plane Spotter - Board-agnostic entry point
 * --------------------------------------------------------------------------
 * Board-specific rendering/touch lives in display_cyd.cpp (TFT_eSPI, the
 * original ESP32-2432S028 "CYD") or display_jc4832.cpp (Arduino_GFX, the
 * ESP32-S3 JC4832W535) — selected per PlatformIO environment via
 * build_src_filter. Everything here calls only the board-agnostic interface
 * declared in shared.h.
 */

#include "shared.h"
#include <WiFi.h>
#include "config.env"

void setup() {
  Serial.begin(115200);

  displaySetup();  // board-specific: touch/display/sprite/canvas init + splash

  for (int i = 0; i < ROUTE_CACHE_SIZE; i++) routeCache[i].valid = false;

  connectWiFiShow();  // board-specific "connecting" feedback, then blocks on connectWiFi()
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  setenv("TZ", TIMEZONE, 1); tzset();

  nearest.valid = false;
  newDataReady = false;
  fetchInProgress = false;

  // Load persisted config (or use defaults)
  prefs.begin("cyd-spotter", false);
  wifiLoadNetworks();
  homeLat = prefs.getDouble("lat", homeLat);
  homeLon = prefs.getDouble("lon", homeLon);
  radarMaxKm = prefs.getFloat("range", radarMaxKm);
  invertColors = prefs.getBool("invert", invertColors);
  brightness = prefs.getUChar("bright", brightness);
  showCallsign = prefs.getBool("callsign", showCallsign);
  showAirline = prefs.getBool("airline", showAirline);
  showSpeed = prefs.getBool("speed", showSpeed);
  showFL = prefs.getBool("fl", showFL);
  showRoute = prefs.getBool("route", showRoute);
  showReg = prefs.getBool("reg", showReg);
  showSquawk = prefs.getBool("squawk", showSquawk);
  showVRate = prefs.getBool("vrate", showVRate);
  showType = prefs.getBool("type", showType);
  showTraces = prefs.getBool("traces", showTraces);
  preferLocalTables = prefs.getBool("tables", preferLocalTables);
  maxBlipsShown = prefs.getUChar("maxblips", maxBlipsShown);
  sweepPeriodSec = prefs.getFloat("swpsec", sweepPeriodSec);
  radarRedrawMs = prefs.getUShort("redrawms", radarRedrawMs);
  fetchIntervalSec = prefs.getUShort("fetchsec", fetchIntervalSec);
  trailMaxSamples = prefs.getUShort("trailsamp", trailMaxSamples);
  trailStaleSec = prefs.getUShort("trailstale", trailStaleSec);
  classNearKm = prefs.getFloat("cnearkm", classNearKm);
  classMaxAltFt = prefs.getUShort("cmaxalt", classMaxAltFt);
  classVrateFpm = prefs.getShort("cvrate", classVrateFpm);
  showAirports = prefs.getBool("showapt", showAirports);
  showTrailKey = prefs.getBool("showkey", showTrailKey);
  showCompass = prefs.getBool("showcomp", showCompass);
  colSweep = prefs.getUShort("c_sweep", colSweep);
  colBlip = prefs.getUShort("c_blip", colBlip);
  colBlipHi = prefs.getUShort("c_bliphi", colBlipHi);
  colRings = prefs.getUShort("c_rings", colRings);
  colAirport = prefs.getUShort("c_airpt", colAirport);
  colTrailDep = prefs.getUShort("c_trdep", colTrailDep);
  colTrailArr = prefs.getUShort("c_trarr", colTrailArr);
  colTrailOver = prefs.getUShort("c_trover", colTrailOver);
  for (uint8_t i = 0; i < FILTER_MAX_RULES; i++) {
    char ken[9], km[8], kt[8], ka[8], kc[8];
    snprintf(ken, sizeof(ken), "fr_en%d", i);
    snprintf(km, sizeof(km), "fr_m%d", i);
    snprintf(kt, sizeof(kt), "fr_t%d", i);
    snprintf(ka, sizeof(ka), "fr_a%d", i);
    snprintf(kc, sizeof(kc), "fr_c%d", i);
    filterRules[i].enabled = prefs.getBool(ken, filterRules[i].enabled);
    filterRules[i].match = prefs.getUChar(km, filterRules[i].match);
    String ft = prefs.getString(kt, filterRules[i].text);
    strncpy(filterRules[i].text, ft.c_str(), 11); filterRules[i].text[11] = '\0';
    filterRules[i].action = prefs.getUChar(ka, filterRules[i].action);
    filterRules[i].color = prefs.getUShort(kc, filterRules[i].color);
  }
  filterQuiet = prefs.getBool("fr_quiet", filterQuiet);
  sweepGlow = prefs.getBool("swpglow", sweepGlow);
  sweepGlowLen = prefs.getUChar("swpglowlen", sweepGlowLen);
  nightDimOn = prefs.getBool("nd_on", nightDimOn);
  nightStartHr = prefs.getUChar("nd_start", nightStartHr);
  nightEndHr = prefs.getUChar("nd_end", nightEndHr);
  nightBrightPct = prefs.getUChar("nd_bright", nightBrightPct);
  autoCycleOn = prefs.getBool("acycle", autoCycleOn);
  autoCycleSec = prefs.getUShort("acyclesec", autoCycleSec);
  applyInvertColors(invertColors);
  applyBrightness(brightness);
  Serial.printf("Config loaded: lat=%.6f lon=%.6f range=%.1f invert=%d bright=%d callsign=%d airline=%d speed=%d fl=%d route=%d reg=%d squawk=%d vrate=%d type=%d traces=%d tables=%d\n",
                homeLat, homeLon, radarMaxKm, invertColors, brightness, showCallsign, showAirline, showSpeed, showFL,
                showRoute, showReg, showSquawk, showVRate, showType, showTraces, preferLocalTables);

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

  // Web server on Core 0 too (see webServerTask in data.cpp) — HTTP requests
  // no longer compete with rendering on Core 1.
  xTaskCreatePinnedToCore(
    webServerTask,     // Function
    "WebSrv",          // Name
    8192,              // Stack (8 KB — HTTP parsing + config page)
    NULL,              // Parameter
    1,                 // Priority
    NULL,              // Handle
    0                  // Core 0
  );
}

void loop() {
  uint32_t now = millis();

  // WiFi health — non-blocking multi-network reconnect + fallback setup AP
  // (the radar keeps running on stale data during outages)
  wifiMaintain();

  // Day/night auto-dim: switch backlight between the configured day level
  // (brightness) and night level on a schedule, checked every 30s.
  static uint32_t lastDimCheck = 0;  static int lastAppliedBright = -1;
  if (!nightDimOn) {
    lastAppliedBright = -1;  // force re-apply if re-enabled
  } else if (now - lastDimCheck > 30000) {
    lastDimCheck = now;
    if (time(nullptr) > 1700000000) {
      time_t t = time(nullptr); struct tm lt; localtime_r(&t, &lt);
      bool night = (nightStartHr <= nightEndHr)
        ? (lt.tm_hour >= (int)nightStartHr && lt.tm_hour < (int)nightEndHr)
        : (lt.tm_hour >= (int)nightStartHr || lt.tm_hour < (int)nightEndHr);
      int target = night ? (int)nightBrightPct : (int)brightness;
      if (target != lastAppliedBright) {
        lastAppliedBright = target;
        applyBrightness((uint8_t)target);
        Serial.printf("[dim] night=%d -> backlight %d%%\n", night, target);
      }
    }
  }

  // Deferred config persistence (NVS writes stay out of the interactive path)
  configMaintain();

  // Auto-cycle (kiosk mode): rotate screens on a timer; any touch pauses
  // cycling for one interval. Screen count comes from the board.
  if (autoCycleOn) {
    uint32_t lastAct = lastScreenSwap > lastTouchMs ? lastScreenSwap : lastTouchMs;
    if (now - lastAct >= (uint32_t)autoCycleSec * 1000UL) {
      screen = (screen + 1) % displayNumScreens();
      lastScreenSwap = now;
    }
  }

  // Weather fetch still runs in loop (it's infrequent, every 10 min)
  if (!firstWeatherDone || now - lastWeatherPoll >= WEATHER_INTERVAL_MS) {
    if (WiFi.status() == WL_CONNECTED) fetchWeather();
    lastWeatherPoll = now; firstWeatherDone = true;
  }

  checkTouch();

  render();
}
