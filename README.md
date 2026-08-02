# CYD28 Plane Spotter ✈️

A real-time aircraft tracker that renders a live radar PPI (Plan Position Indicator), shows nearby aircraft details, and provides weather & system status. Supports three boards from one codebase: the **ESP32 Cheap Yellow Display (CYD)** (2.8" 240×320, full 5-screen UI), the **JC4832W535** (ESP32-S3, 3.5" 480×320 QSPI display, full 4-screen UI), and the **ESP32-S3-Touch-LCD-7B** (Waveshare, ESP32-S3, 7" 1024×600 RGB-parallel display — currently a minimal hardware bring-up, not yet the full UI; see Roadmap).

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
- **Second board: JC4832W535** — `main.cpp` split into a board-agnostic data layer (`shared.h`/`data.cpp`) and per-board rendering (`display_cyd.cpp` for the original TFT_eSPI/CYD code, `display_jc4832.cpp` new), selected via separate PlatformIO environments (`pio run -e cyd` / `-e jc4832`). The JC4832W535 uses a QSPI/AXS15231B display TFT_eSPI can't drive at all, so it runs on Arduino_GFX instead — a genuinely different display library, not a config change.
- **Full UI port to the JC4832W535** — `display_jc4832.cpp` now implements the same Target Intel / Top 5 / Radar / Weather & System screen set as the CYD, ported to Arduino_GFX (no sprites — a single full-panel `Arduino_Canvas`, `setCursor()+print()` instead of `drawString()`, `getTextBounds()` instead of `textWidth()`/`fontHeight()`) and laid out fresh for the 480×320 canvas. The CYD's two separate radar variants (compact "Radar PPI" + borderless "Radar Full") are consolidated into one bigger Radar screen here, since this board's wider canvas doesn't have the space constraint that motivated splitting them on the CYD.
- **Third board: ESP32-S3-Touch-LCD-7B** — a Waveshare 7" 1024×600 board added the same way (`display_lcd7b.cpp`, `pio run -e lcd7b`). Unlike the other two boards, this one's ST7701 panel is direct RGB-parallel (no QSPI/SPI command interface at all), and touch reset/backlight run through a small I2C GPIO-expander rather than direct GPIOs. Currently a minimal bring-up (see Roadmap for the full UI port).

## Design Decisions

The changes above are the *what*; a few of them involved real tradeoffs worth explaining.

- **Dark Mode is a hardware color invert, not a second color palette.** `tft.invertDisplay()` flips every color the ILI9341 panel renders — background, text, the radar sprite, icons — with a single call and zero changes to any drawing code. The alternative (a hand-built light palette threaded through every screen) would look more deliberately designed and preserve color *meaning* (e.g. the red "NO TARGET IN RANGE" text currently becomes cyan when inverted, since it's a literal color complement, not a redesign). That alternative touches ~70 hardcoded color constants across every screen for a mostly-cosmetic gain, and a broad multi-site refactor like that is exactly the kind of change that risks introducing subtle rendering bugs (a sibling CYD project evaluated before this one had real flicker and text-ghosting bugs caused by direct-to-panel drawing without careful erase/redraw handling — a cautionary example, not something that happened in this repo). One function call won on cost, at the price of not being a "proper" light theme.

- **Route and airline data went through two designs before landing on adsbdb.com.** The first version used hexdb.io (route only, returns ICAO airport codes like `RKSI`) plus a hand-typed ICAO→IATA airport table and an airline-callsign-prefix table — both curated from memory, and both turned out to have real gaps and at least one wrong entry. Before fixing that by just expanding the tables, an algorithmic shortcut was identified (North American airports overwhelmingly satisfy IATA = last 3 letters of the ICAO code — `KLAX`→`LAX`, `CYYZ`→`YYZ` — so they don't need table entries at all) to make a bigger table fit the flash budget. That whole path became unnecessary once **adsbdb.com**'s `/v0/callsign/{callsign}` endpoint turned out to return the airline name *and* both airports' real IATA codes directly, globally, in one call that's also faster than the hexdb.io lookup it replaced (~1s vs. 2-3s). The static airline table wasn't deleted — it's kept as a small fallback for the rare callsign adsbdb.com doesn't recognize, since it costs nothing to keep and occasionally still helps.

- **Radar blip labels are individually toggleable and default off, deliberately.** Earlier iterations were more opinionated (a fixed layout, or one field with an automatic fallback to another). The user's explicit direction was: don't decide for me, make everything a switch, and start with nothing forced on. That required changing the label-drawing code from fixed line positions to a dynamic stack — count how many fields are actually enabled *and* have data for a given aircraft, size the label to fit, and clamp its position so a taller stack still fits inside the radar sprite instead of overlapping itself (the old fixed-position clamp would do that once more than 2-3 lines were in play).

- **Route lookups for every radar blip use a budgeted background fetch, not a synchronous one.** Each route/airline lookup takes roughly 1-3 seconds. With up to 20 aircraft visible at once, fetching all of them on every poll cycle would block for up to a minute and blow past the 10-second poll interval. Instead, each cycle does a free cache-only sweep across all blips (picks up anything already known, no network cost) followed by a capped live-fetch pass (6 new lookups per cycle) for aircraft not yet seen. Coverage converges within a couple of cycles for aircraft that stay in view, and worst-case added latency per cycle stays bounded regardless of how much traffic is nearby.

- **The flash budget shaped what "comprehensive" could mean.** This board has roughly 270KB of flash headroom. A truly exhaustive airline/airport dataset (the kind OpenFlights or a full ICAO registry provides — thousands of entries covering every charter operator and small airfield) doesn't fit. The fallback tables here are intentionally scoped to major, verified entries rather than attempting full global coverage locally — real global coverage instead comes from the live adsbdb.com lookup, with the local table only covering its gaps.

- **A much larger, verified dataset exists in the source but isn't wired up.** `src/data.cpp` also contains `AIRLINE_CODES_FULL` (303 entries) and `AIRPORT_CODES_ICAO_IATA` (319 entries) — sourced from raw Wikipedia airline/airport code wikitext, parsed directly and cross-checked entry by entry (not summarized, after an early summarizer pass hallucinated a wrong code). Neither is referenced by any function, so the linker's dead-code elimination strips both entirely — confirmed via the linked binary's symbol table, flash usage is byte-identical with or without them present. `AIRLINE_CODES_FULL` is a straightforward drop-in upgrade for the active fallback table (~6.9KB once wired up) whenever broader offline coverage is wanted. `AIRPORT_CODES_ICAO_IATA` (~5.3KB) currently has no use case — the only code path that needed ICAO→IATA conversion was the old hexdb.io route lookup, which adsbdb.com's direct IATA codes made obsolete — it'd only become useful again alongside a secondary/backup route data source.

- **Two permanent sprites, not one resized on the fly.** The Radar Full screen originally shared `radarSpr` with the small Radar PPI screen, resizing it (`deleteSprite()` + `createSprite()`) on every transition between the two screens rather than permanently reserving memory for both sizes at once. In practice this fragmented the heap badly enough that later allocations silently failed — both radar screens stopped drawing at all, and shortly after, TLS/HTTPS itself started failing with `X509 - Allocation of memory failed`. Fixed by allocating two separate sprites once at boot and never freeing either — `radarSpr` (200×200) and `radarSprFull` (234×234), both switched from 16-bit to 8-bit color to keep their combined footprint small enough to leave real headroom for WiFi/TLS buffers. Verified with 30+ seconds of continuous successful HTTPS calls afterward, no further allocation failures. On a single-heap, no-PSRAM board like this one, permanent fixed-size allocations made once at boot are more predictable than repeated resize/free cycles at runtime, even though they use somewhat more memory at rest.

- **JC4832W535 bring-up needed native resolution + software rotation, not the panel's marketed landscape dimensions.** Telling the AXS15231B driver its native resolution was 480×320 (this board's marketed spec) directly produced a garbled bottom third of the screen — data was reaching the panel, but column/row addressing was wrong. The only proven-working reference found for this chip family (the sibling JC3248W535 board) instead keeps the driver at its electrical native 320×480 and reaches landscape via software `setRotation(1)`. Matching that exactly fixed it. Also required pinning `Arduino_GFX` to `1.4.5` — newer releases assume ESP32 Arduino core 3.x's `esp32-hal-periman.h`, which the core 2.0.14 both environments currently use doesn't have; this only affects the `jc4832` environment; the CYD's `TFT_eSPI` toolchain is untouched.

- **The JC4832W535's Radar PPI and Radar Full became one screen, not two.** The CYD has both because its 320×240 canvas is too tight for one radar view to have both a header and a large circle at once. The JC4832W535's 480×320 canvas doesn't have that constraint, so porting both variants over would have just replicated a CYD-specific workaround rather than done "the full port" in spirit — one Radar screen here has both a header and a bigger circle (R=145) than either CYD variant achieves.

- **ESP32-S3-Touch-LCD-7B bring-up was built from the vendor's own reference code, not guessed pin/timing values — but still took three real bugs to get right.** This board's ST7701 panel is direct RGB-parallel — 16 data lines plus HSYNC/VSYNC/DE/PCLK, no command interface, no forgiving fallback the way a wrong QSPI init just fails cleanly. Guessing here risked repeating the JC4832W535's garbled-display bug, so every pin and timing value in `display_lcd7b.cpp` was sourced directly from Waveshare's own official example repo (`waveshareteam/ESP32-S3-Touch-LCD-7B`) rather than assumption. Two findings from that reference shaped the implementation: the panel needs **no vendor init sequence at all** (Waveshare's own code calls the raw ESP-IDF RGB panel API with no ST7701 register writes — it self-configures into RGB passthrough, matched here via Arduino_GFX's `Arduino_RGB_Display` in its "bare" mode), and touch reset/backlight run through a small I2C GPIO-expander (`IOExtension`, addr `0x24`) rather than direct GPIOs, discovered from the vendor's `io_extension.c`.
  - **Bug 1 — rolling/jumping image, not a clean failure.** First flash produced a picture that never settled, like a TV tuned between channels. Root cause: the vendor's 30MHz pixel clock exceeds the sustainable rate for Octal PSRAM @ 80MHz on a *bounce-buffer-less* RGB panel setup (~22MHz ceiling) — the vendor's own ESP-IDF example uses a bounce buffer (`bounce_buffer_size_px`) to work around exactly this, but Arduino_GFX's `Arduino_ESP32RGBPanel` doesn't expose that option at all, so the framebuffer is read from PSRAM directly by DMA at the full requested pixel rate. Dropped to 16MHz (matching a confirmed-working Arduino_GFX config for a pin-compatible sibling Waveshare board, and close to the library's own internal 12MHz default for non-Quad-PSRAM boards) — fixed it completely. A `hsync_polarity`/`vsync_polarity` change was tried first and didn't help; wasted effort, since `Arduino_ESP32RGBPanel::getFrameBuffer()` writes those params directly into the raw `LCD_CAM.lcd_ctrl2` idle-pol hardware registers *after* the panel is already initialized, overriding whatever the struct-based `hsync_idle_low`/`vsync_idle_low` flags computed — so reasoning about matching the vendor's ESP-IDF struct defaults was moot from the start. `0/0` (the original value) turned out to match two independent working Arduino_GFX/LovyanGFX configs for this exact pin-compatible board family and was never actually the problem.
  - **Bug 2 — full-screen flicker once the image was stable.** `Arduino_RGB_Display`'s `auto_flush` mode writes straight into the live, continuously-scanned framebuffer (no back buffer) — clearing the whole screen every render cycle is visible as a black flash each time. Fixed by clearing once and using opaque `(fg, bg)` text color so redraws overwrite their own old footprint in place.
  - **Bug 3 — touch detected but coordinates were nonsensical (tens of thousands, not 0–1023/0–599).** The GT911 point-1 register layout was assumed to start with a track-ID byte (`buf[0]`) followed by X/Y at `buf[1..4]`, copied from the AXS15231B driver's shape without re-verifying against the GT911's actual register map. The real layout has no leading byte at all — X low/high then Y low/high start immediately at `0x8150`; track ID is a separate field at `0x8157`, outside the 6 bytes actually needed. The off-by-one silently produced plausible-looking garbage instead of an obvious failure, so it took reading the GT911 register map directly (not just trusting the AXS15231B-derived code shape) to catch it.

- **GT911 touch on the LCD-7B is a hand-rolled driver, not the vendor's `gt911.cpp`.** That file turned out to be the generic ESP-IDF `esp_lcd_touch_gt911` component — sleep modes, interrupt callbacks, and other machinery this project doesn't need. Instead `GT911_touch.{h,cpp}` implements just the well-documented public GT911 register map (status at `0x814E`, point data from `0x8150`) directly over `Wire`, matching the project's existing pattern for the JC4832W535's `AXS15231B_touch.{h,cpp}`. A `GET /gt911debug` route (registered in `display_lcd7b.cpp`) dumps recent status-register polls and the last decoded touch coordinates — added because this board's native USB-CDC serial proved unreliable across resets during bring-up (needs a real power-cycle, not a soft/RTS-triggered reset, to reliably re-enumerate), so this diagnostic works over WiFi instead. Left in intentionally, same as the project's other diagnostic logging.

## Features

- **Radar PPI** — rotating sweep line with aircraft blips; each label's fields (callsign, altitude, route, airline, registration, squawk, vertical rate, aircraft type, speed) are independently toggleable from the web config page. Configurable range with on-screen +/- buttons.
- **Radar Full** — a second, larger radar view: no header/clock, the circle sized to fill nearly the full screen height instead of sharing space with a side info column. Weather (icon, temp, wind) in the bottom-left margin; range value and +/- buttons in the right margin. Labels use collision avoidance so two different aircraft's labels don't draw on top of each other (the small Radar PPI screen shares this same logic).
- **Target Intel** — detailed view of the nearest aircraft: callsign, route (origin → destination), country, distance/bearing, altitude, speed, and heading arrow.
- **Top 5 In Range** — scrollable list of the 5 closest flights with route and altitude info.
- **Weather & System** — live temperature, humidity, and wind from Open-Meteo, plus system stats (uptime, WiFi RSSI, free RAM, API request counters).
- **Web Configuration** — built-in web server to change home location, radar range, and dark mode from any browser on the same network.
- **Over-the-air firmware updates** — a "Firmware Update" upload form on the same web config page flashes a new `.bin` over WiFi (`POST /update`, backed by the standard ESP32 `Update` library) and reboots into it — no USB cable needed after the first flash. Works identically on all three boards, since it's part of the shared `data.cpp` web server.
- **Dual-core architecture** — HTTP/API calls run on Core 0 (FreeRTOS task), UI rendering and touch handling run on Core 1, keeping the display smooth.

## Hardware

**CYD (`env:cyd`)** — full 5-screen UI
- **Board**: ESP32-2432S028 (CYD28) — 2.8" TFT with ILI9341 driver
- **Touch**: XPT2046 resistive touch controller
- **Display**: 240×320 pixels, rotation 1 (landscape)

**JC4832W535 (`env:jc4832`)** — full 4-screen UI (Target Intel, Top 5, Radar, Weather & System)
- **Board**: ESP32-S3-WROOM-1-N16R8 (16MB flash, 8MB PSRAM), Guition JC4832W535
- **Display**: AXS15231B over QSPI, native 320×480, software-rotated to 480×320 landscape
- **Touch**: AXS15231B integrated capacitive touch over I2C (vendored driver, `src/AXS15231B_touch.{h,cpp}`)

**ESP32-S3-Touch-LCD-7B (`env:lcd7b`)** — minimal bring-up (WiFi/aircraft-count/nearest-callsign screen; full UI is a Roadmap item)
- **Board**: ESP32-S3-WROOM-1-N16R8 (16MB flash, 8MB PSRAM), Waveshare ESP32-S3-Touch-LCD-7B
- **Display**: ST7701, direct RGB-parallel (16-bit data bus + HSYNC/VSYNC/DE/PCLK), 1024×600, no vendor init sequence needed
- **Touch**: GT911 capacitive touch over I2C (hand-rolled driver, `src/GT911_touch.{h,cpp}`); reset and backlight run through an I2C GPIO-expander (`src/IOExtension.{h,cpp}`), not direct GPIOs

## Data Sources

| Source | Purpose |
|--------|---------|
| [adsb.lol](https://api.adsb.lol/) | Real-time ADS-B aircraft states (free, keyless — swapped from OpenSky due to persistent anonymous-tier 429s) |
| [adsbdb.com](https://api.adsbdb.com/) | Callsign → airline name + route (origin/destination IATA codes) |
| [Open-Meteo](https://open-meteo.com/) | Weather (temperature, humidity, wind speed) |

## Project Structure

```
CYD28/
├── platformio.ini          # [env:cyd], [env:jc4832], [env:lcd7b] build configuration
├── config.env.example      # Configuration template (copy to config.env)
├── boards/
│   └── esp32-s3-n16r8v.json  # Custom board def for the JC4832W535/LCD-7B's ESP32-S3 module
├── .gitignore
├── README.md
└── src/
    ├── main.cpp             # Board-agnostic entry point (setup/loop only)
    ├── shared.h              # Structs, config globals, math helpers, board interface
    ├── data.cpp               # WiFi, adsb.lol/adsbdb.com/weather fetching, web config server
    ├── display_cyd.cpp        # CYD-only: TFT_eSPI rendering + XPT2046 touch
    ├── display_jc4832.cpp     # JC4832W535-only: Arduino_GFX rendering + I2C touch
    ├── AXS15231B_touch.{h,cpp}  # Vendored touch driver for the JC4832W535
    ├── display_lcd7b.cpp      # ESP32-S3-Touch-LCD-7B-only: Arduino_GFX RGB-panel rendering + GT911 touch
    ├── IOExtension.{h,cpp}     # I2C GPIO-expander driver for the LCD-7B (backlight, touch reset)
    ├── GT911_touch.{h,cpp}     # Hand-rolled GT911 touch driver for the LCD-7B
    └── config.env            # Your private config (gitignored)
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
5. **Build & upload** — specify which board:
   ```
   pio run -e cyd -t upload      # CYD (ESP32-2432S028)
   pio run -e jc4832 -t upload   # JC4832W535 (ESP32-S3)
   pio run -e lcd7b -t upload    # ESP32-S3-Touch-LCD-7B (ESP32-S3)
   ```
   All three boards share `src/config.env` — no per-board config needed.
6. Once connected, find the device IP in the serial monitor and open it in a browser to change settings live (CYD and JC4832W535 reflect settings back on-screen; the LCD-7B's web server runs too, but there's no on-screen UI yet to reflect the settings back — see Roadmap).

## Usage

- **Tap the screen** to cycle through the 5 screens (Target Intel → Top 5 → Radar PPI → Radar Full → Weather & System).
- **On the Radar screen**, tap the **+/- buttons** at the bottom-left to adjust range (10–200 km) — this also sets the aircraft data fetch radius.
- **Open the device IP** in a browser to configure location, range, Dark Mode, and which fields show on radar blip labels.
- **Firmware updates over WiFi** — same page, "Firmware Update" card: pick a `.bin` built via `pio run -e <board>` (from `.pio/build/<board>/firmware.bin` — the plain app binary, not a merged image) and upload. The device flashes it into the inactive OTA slot and reboots. There's no board-identity check on the uploaded file, so double-check you're uploading the right board's build.

## Roadmap

Ideas noted for future work, not yet implemented:

- **Arrivals/departures board per nearby airport** — a dedicated screen (one per airport, e.g. GMP and ICN for the primary dev setup) listing upcoming arrivals and departures at that specific airport, rather than the existing Top 5/Target Intel screens which are organized by distance from home location instead of by airport.
- **On-hardware touch calibration for the JC4832W535** — the current touch offsets are a placeholder spanning the full native panel range; `checkTouch()` logs raw coordinates to Serial specifically so real calibration data can be gathered and the offsets tuned.
- **WiFi provisioning via AP-mode + QR code** — when the device has no working WiFi credentials, boot into a temporary AP mode and show a QR code on-screen encoding the standard `WIFI:` connection format, so scanning it with a phone connects directly to the device's setup network (no manually typing the SSID/password) and lands on the existing captive-portal-style config page to enter real credentials. Not started.
- **Radar label-keying scheme** — an alternative to the current full-text blip labels: short codes (A, B, C… or 1, 2, 3…) drawn directly on the radar next to each blip, with a key/legend panel off to the side mapping each code to the full aircraft details (callsign, altitude, route, etc.) currently shown inline. Would declutter a busy radar with many toggled-on fields at the cost of an extra lookup step. Explicitly backlogged, not started — flagged during LCD-7B radar work as a possible answer to label crowding, but not the immediate priority.

## Dependencies

**CYD (`env:cyd`)**
- [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) — TFT display driver
- [XPT2046_Touchscreen](https://github.com/PaulStoffregen/XPT2046_Touchscreen) — Touch controller

**JC4832W535 (`env:jc4832`) and ESP32-S3-Touch-LCD-7B (`env:lcd7b`)**
- [Arduino_GFX](https://github.com/moononournation/Arduino_GFX) — pinned to `1.4.5`; newer releases require ESP32 Arduino core 3.x, which this project doesn't currently use (see Design Decisions)

**All three**
- [ArduinoJson](https://arduinojson.org/) — JSON parsing

All managed automatically via PlatformIO `lib_deps`, per environment.

## License

MIT
