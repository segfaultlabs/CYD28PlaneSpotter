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
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(400); }
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
void initWebServer() {
  // GET / — serve config page
  server.on("/", []() {
    // Snapshot current values
    double hLat, hLon; float rMax; bool inv; uint8_t bright;
    bool showCS, showAir, showSpd, showFlt, showRte, showRg, showSq, showVr, showTy, showTr, preferTables;
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    hLat = homeLat; hLon = homeLon; rMax = radarMaxKm; inv = invertColors; bright = brightness;
    showCS = showCallsign; showAir = showAirline; showSpd = showSpeed; showFlt = showFL; showRte = showRoute;
    showRg = showReg; showSq = showSquawk; showVr = showVRate; showTy = showType; showTr = showTraces; preferTables = preferLocalTables;
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
  input[type=range]{width:100%;padding:0;accent-color:#00ff88}
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
         '&tables='+(document.getElementById('tables').checked?'1':'0'));
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

    // Basic validation
    if (newLat < -90 || newLat > 90 || newLon < -180 || newLon > 180 ||
        newRng <= 0 || newRng > 500) {
      server.send(400, "text/plain", "Invalid values");
      return;
    }
    if (newBrightness < 1) newBrightness = 1;
    if (newBrightness > 100) newBrightness = 100;

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

    Serial.printf("Config saved: lat=%.6f lon=%.6f range=%.1f invert=%d callsign=%d airline=%d speed=%d fl=%d route=%d reg=%d squawk=%d vrate=%d type=%d traces=%d tables=%d\n",
                  newLat, newLon, newRng, newInvert, newShowCallsign, newShowAirline, newShowSpeed, newShowFL,
                  newShowRoute, newShowReg, newShowSquawk, newShowVRate, newShowType, newShowTraces, newPreferTables);
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
