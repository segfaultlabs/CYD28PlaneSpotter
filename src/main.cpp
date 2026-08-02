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
  showTraces = prefs.getBool("traces", showTraces);
  preferLocalTables = prefs.getBool("tables", preferLocalTables);
  applyInvertColors(invertColors);
  Serial.printf("Config loaded: lat=%.6f lon=%.6f range=%.1f invert=%d callsign=%d airline=%d speed=%d fl=%d route=%d reg=%d squawk=%d vrate=%d type=%d traces=%d tables=%d\n",
                homeLat, homeLon, radarMaxKm, invertColors, showCallsign, showAirline, showSpeed, showFL,
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
}

void loop() {
  uint32_t now = millis();

  // WiFi health — reconnect if needed (shows status, so must be Core 1)
  if (WiFi.status() != WL_CONNECTED) connectWiFiShow();

  // Weather fetch still runs in loop (it's infrequent, every 10 min)
  if (!firstWeatherDone || now - lastWeatherPoll >= WEATHER_INTERVAL_MS) {
    if (WiFi.status() == WL_CONNECTED) fetchWeather();
    lastWeatherPoll = now; firstWeatherDone = true;
  }

  checkTouch();

  // Serve web config requests
  server.handleClient();

  render();
}
