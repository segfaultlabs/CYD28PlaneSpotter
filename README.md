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

## Design Decisions

The changes above are the *what*; a few of them involved real tradeoffs worth explaining.

- **Dark Mode is a hardware color invert, not a second color palette.** `tft.invertDisplay()` flips every color the ILI9341 panel renders — background, text, the radar sprite, icons — with a single call and zero changes to any drawing code. The alternative (a hand-built light palette threaded through every screen) would look more deliberately designed and preserve color *meaning* (e.g. the red "NO TARGET IN RANGE" text currently becomes cyan when inverted, since it's a literal color complement, not a redesign). That alternative touches ~70 hardcoded color constants across every screen for a mostly-cosmetic gain, and a broad multi-site refactor like that is exactly the kind of change that risks introducing subtle rendering bugs (a sibling CYD project evaluated before this one had real flicker and text-ghosting bugs caused by direct-to-panel drawing without careful erase/redraw handling — a cautionary example, not something that happened in this repo). One function call won on cost, at the price of not being a "proper" light theme.

- **Route and airline data went through two designs before landing on adsbdb.com.** The first version used hexdb.io (route only, returns ICAO airport codes like `RKSI`) plus a hand-typed ICAO→IATA airport table and an airline-callsign-prefix table — both curated from memory, and both turned out to have real gaps and at least one wrong entry. Before fixing that by just expanding the tables, an algorithmic shortcut was identified (North American airports overwhelmingly satisfy IATA = last 3 letters of the ICAO code — `KLAX`→`LAX`, `CYYZ`→`YYZ` — so they don't need table entries at all) to make a bigger table fit the flash budget. That whole path became unnecessary once **adsbdb.com**'s `/v0/callsign/{callsign}` endpoint turned out to return the airline name *and* both airports' real IATA codes directly, globally, in one call that's also faster than the hexdb.io lookup it replaced (~1s vs. 2-3s). The static airline table wasn't deleted — it's kept as a small fallback for the rare callsign adsbdb.com doesn't recognize, since it costs nothing to keep and occasionally still helps.

- **Radar blip labels are individually toggleable and default off, deliberately.** Earlier iterations were more opinionated (a fixed layout, or one field with an automatic fallback to another). The user's explicit direction was: don't decide for me, make everything a switch, and start with nothing forced on. That required changing the label-drawing code from fixed line positions to a dynamic stack — count how many fields are actually enabled *and* have data for a given aircraft, size the label to fit, and clamp its position so a taller stack still fits inside the radar sprite instead of overlapping itself (the old fixed-position clamp would do that once more than 2-3 lines were in play).

- **Route lookups for every radar blip use a budgeted background fetch, not a synchronous one.** Each route/airline lookup takes roughly 1-3 seconds. With up to 20 aircraft visible at once, fetching all of them on every poll cycle would block for up to a minute and blow past the 10-second poll interval. Instead, each cycle does a free cache-only sweep across all blips (picks up anything already known, no network cost) followed by a capped live-fetch pass (6 new lookups per cycle) for aircraft not yet seen. Coverage converges within a couple of cycles for aircraft that stay in view, and worst-case added latency per cycle stays bounded regardless of how much traffic is nearby.

- **The flash budget shaped what "comprehensive" could mean.** This board has roughly 270KB of flash headroom. A truly exhaustive airline/airport dataset (the kind OpenFlights or a full ICAO registry provides — thousands of entries covering every charter operator and small airfield) doesn't fit. The fallback tables here are intentionally scoped to major, verified entries rather than attempting full global coverage locally — real global coverage instead comes from the live adsbdb.com lookup, with the local table only covering its gaps.

- **A much larger, verified dataset exists in the source but isn't wired up.** `src/main.cpp` also contains `AIRLINE_CODES_FULL` (303 entries) and `AIRPORT_CODES_ICAO_IATA` (319 entries) — sourced from raw Wikipedia airline/airport code wikitext, parsed directly and cross-checked entry by entry (not summarized, after an early summarizer pass hallucinated a wrong code). Neither is referenced by any function, so the linker's dead-code elimination strips both entirely — confirmed via the linked binary's symbol table, flash usage is byte-identical with or without them present. `AIRLINE_CODES_FULL` is a straightforward drop-in upgrade for the active fallback table (~6.9KB once wired up) whenever broader offline coverage is wanted. `AIRPORT_CODES_ICAO_IATA` (~5.3KB) currently has no use case — the only code path that needed ICAO→IATA conversion was the old hexdb.io route lookup, which adsbdb.com's direct IATA codes made obsolete — it'd only become useful again alongside a secondary/backup route data source.

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
