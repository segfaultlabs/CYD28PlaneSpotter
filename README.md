# CYD28 Plane Spotter ✈️

A real-time aircraft tracker for the **ESP32 Cheap Yellow Display (CYD)** that renders a live radar PPI (Plan Position Indicator), shows nearby aircraft details, and provides weather & system status — all on a 2.8" 240×320 TFT touchscreen.

## Changes from Upstream

This is a fork of [ppoinha/CYD28PlaneSpotter](https://github.com/ppoinha/CYD28PlaneSpotter). Changes made in this fork:

- **Data source: OpenSky → adsb.lol** — OpenSky's anonymous API tier was returning HTTP 429 on every single request regardless of polling interval, and this repo's OpenSky client-credential config fields were never actually wired up to any auth flow, so there was no way to get past it. Switched to [adsb.lol](https://api.adsb.lol/), a free, keyless API. As part of the swap, the fetch radius is now derived directly from the on-screen radar range (km) — upstream used a separate, disconnected "search radius in degrees" field that could silently cap data coverage well short of whatever range was selected on screen.
- **Display driver fix** — corrected `ST7789_DRIVER` to `ILI9341_2_DRIVER` in `platformio.ini`. This board revision uses an ILI9341 panel, not ST7789; upstream's setting produced wrong/garbled colors on this hardware.
- **Plane / helicopter icons** — radar blips render as a small heading-oriented plane glyph (fuselage/wings/tailfin), or a rotor-cross glyph for rotorcraft (ADS-B category `A7`), instead of a plain dot/square.
- **Weather icon** — the Radar PPI screen shows a small vector weather icon (sun/cloud/rain/snow/storm, derived from Open-Meteo's WMO weather code) next to the temp/humidity/wind readout.
- **Dark Mode toggle** — a switch on the web config page, backed by `tft.invertDisplay()` and persisted to NVS, so the color scheme can be flipped live without reflashing.
- **Fully configurable radar blip labels** — every field (flight number, altitude, route, airline, registration, squawk, vertical rate, aircraft type, speed) is its own independent on/off switch on the web config page. Nothing is forced on; if everything's off, no label draws at all.
- **Route + airline data: hexdb.io → adsbdb.com** — switched the per-aircraft lookup to [adsbdb.com](https://api.adsbdb.com/)'s `/v0/callsign/{callsign}` endpoint, which returns the airline name and both origin/destination airport **IATA** codes (e.g. `ICN`, not the ICAO code `RKSI`) in a single call — faster (~1s vs. hexdb.io's 2-3s) and better data than the two sources (plus a static ICAO→IATA table) this used to require. A small local airline-code table is kept only as a fallback for the rare callsign adsbdb.com doesn't recognize.
- **Diagnostic logging** — HTTP status/timing/heap logging around the aircraft fetch, route/airline lookup, and weather calls, added while debugging the original OpenSky failures and left in intentionally for ongoing visibility.

## Features

- **Radar PPI** — rotating sweep line with aircraft blips; each label's fields (callsign, altitude, route, airline, registration, squawk, vertical rate, aircraft type, speed) are independently toggleable from the web config page. Configurable range with on-screen +/- buttons.
- **Target Intel** — detailed view of the nearest aircraft: callsign, route (origin → destination), country, distance/bearing, altitude, speed, and heading arrow.
- **Top 5 In Range** — scrollable list of the 5 closest flights with route and altitude info.
- **Weather & System** — live temperature, humidity, and wind from Open-Meteo, plus system stats (uptime, WiFi RSSI, free RAM, API request counters).
- **Web Configuration** — built-in web server to change home location, radar range, and dark mode from any browser on the same network.
- **Dual-core architecture** — HTTP/API calls run on Core 0 (FreeRTOS task), UI rendering and touch handling run on Core 1, keeping the display smooth.

## Hardware

- **Board**: ESP32-2432S028 (CYD28) — 2.8" TFT with ILI9341 driver
- **Touch**: XPT2046 resistive touch controller
- **Display**: 240×320 pixels, rotation 1 (landscape)

## Data Sources

| Source | Purpose |
|--------|---------|
| [adsb.lol](https://api.adsb.lol/) | Real-time ADS-B aircraft states (free, keyless — swapped from OpenSky due to persistent anonymous-tier 429s) |
| [adsbdb.com](https://api.adsbdb.com/) | Callsign → airline name + route (origin/destination IATA codes) |
| [Open-Meteo](https://open-meteo.com/) | Weather (temperature, humidity, wind speed) |

## Project Structure

```
CYD28/
├── platformio.ini          # PlatformIO build configuration
├── config.env.example      # Configuration template (copy to config.env)
├── .gitignore
├── README.md
└── src/
    ├── main.cpp            # Main application code
    └── config.env          # Your private config (gitignored)
```

## Setup

1. **Clone & open** in VS Code with the PlatformIO extension.
2. **Copy the config template**:
   ```
   cp config.env.example src/config.env
   ```
3. **Edit `src/config.env`** with your WiFi credentials:
   ```c
   #define WIFI_SSID "YourWiFi"
   #define WIFI_PASS "YourPassword"
   ```
   (No API key needed — aircraft data comes from adsb.lol, which is free and keyless.)
4. **Adjust defaults** for your home location if desired:
   ```c
   #define DEFAULT_HOME_LAT      47.5774   // your latitude
   #define DEFAULT_HOME_LON      8.5212    // your longitude
   ```
5. **Build & upload**:
   ```
   pio run --target upload
   ```
6. Once connected, find the device IP in the serial monitor and open it in a browser to change settings live.

## Usage

- **Tap the screen** to cycle through the 4 screens.
- **On the Radar screen**, tap the **+/- buttons** at the bottom-left to adjust range (10–200 km) — this also sets the aircraft data fetch radius.
- **Open the device IP** in a browser to configure location, range, Dark Mode, and which fields show on radar blip labels.

## Dependencies

- [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) — TFT display driver
- [ArduinoJson](https://arduinojson.org/) — JSON parsing
- [XPT2046_Touchscreen](https://github.com/PaulStoffregen/XPT2046_Touchscreen) — Touch controller

All managed automatically via PlatformIO `lib_deps`.

## License

MIT
