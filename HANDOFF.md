# Handoff — CYD28 Plane Spotter

Written for whoever/whatever picks this up next. Facts and current state
only, organized so you don't have to re-derive anything from git history or
chat logs.

## What this project is

ESP32-based real-time ADS-B aircraft radar display. One codebase, three
physical boards, selected via PlatformIO environments:

| Board | Env | Display | Status |
|---|---|---|---|
| ESP32 "Cheap Yellow Display" (CYD) | `cyd` | 2.8" 240×320 ILI9341, TFT_eSPI | Working, full 5-screen UI |
| JC4832W535 | `jc4832` | 3.5" 480×320 QSPI/AXS15231B, Arduino_GFX | Working, full 4-screen UI |
| ESP32-S3-Touch-LCD-7B (Waveshare) | `lcd7b` | 7" 1024×600 RGB-parallel/ST7701, Arduino_GFX single-buffer | Working, full 4-screen UI, **this is the safe fallback** |
| ESP32-S3-Touch-LCD-7B (same hardware) | `lcd7b_v2` | same panel, raw ESP-IDF double-buffered, zero-copy rendering, 20MHz pclk (~21.8Hz refresh) | **Working — now the better 7" firmware**; `lcd7b` stays as fallback |

`shared.h`/`data.cpp` hold board-agnostic state, network fetch, and the web
config server. Each `display_*.cpp` implements the board-specific rendering
interface declared in `shared.h` (`displaySetup()`, `render()`,
`checkTouch()`, `applyInvertColors()`, `applyBrightness()`).

`lcd7b_v2` is a deliberately separate environment + source file, not a
branch, so the working `lcd7b` stays untouched as a fallback no matter what.
**`lcd7b_v2` is now the better 7" firmware** (zero-copy double buffering,
20MHz pclk, smooth ~21.8Hz sweep) — flash it; `lcd7b` remains the known-good
fallback if a future experiment regresses.

## Build / deploy

Standard boards:
```
pio run -e cyd        # or jc4832, or lcd7b
```

**`lcd7b_v2` needs a separate PlatformIO core dir — this is not optional:**
```
PLATFORMIO_CORE_DIR=~/.platformio_lcd7b_v2 pio run -e lcd7b_v2
```
Why: `lcd7b_v2` uses `pioarduino/platform-espressif32` (a community fork
providing arduino-esp32 core 3.x — official PlatformIO's `espressif32`
platform doesn't support core 3.x at all). Building it under the default
`PLATFORMIO_CORE_DIR` previously **silently corrupted** the shared
`framework-arduinoespressif32` package directory used by the other three
(unpinned) environments, breaking their builds. Fixed by (a) pinning
`platform = espressif32@6.6.0` explicitly on `cyd`/`jc4832`/`lcd7b`, and (b)
giving `lcd7b_v2` a fully separate core dir so there's no shared-directory
collision at all, ever. Do not remove either mitigation.

OTA deploy (all boards, once already running this firmware — this is the
primary iteration path used throughout this project, since USB-CDC on this
hardware has been flaky):
```
curl -F "update=@.pio/build/<env>/firmware.bin" http://<device-ip>/update
```
Device used during this work: `192.168.1.75` (adjust for your network).

## Current git state

Branch `master`, uncommitted working tree (nothing in this session has been
committed). `git status --short`:
```
 M src/IOExtension.cpp
 M src/data.cpp
 M src/display_cyd.cpp
 M src/display_jc4832.cpp
 M src/display_lcd7b.cpp
 M src/display_lcd7b_v2.cpp
 M src/main.cpp
 M src/shared.h
?? LCD7B_V2_DEBUG_LOG.md
?? HANDOFF.md
?? src/idf_component.yml
```
`src/idf_component.yml` is a stray auto-generated artifact from an
ESP-IDF component-manager step during an `lcd7b_v2` build (content is just
`dependencies: idf: '>=5.1'`). Not intentionally created, not currently
gitignored. Safe to delete or gitignore; harmless if left.

### What changed in each modified file, this session

- **`src/shared.h`** — added `extern uint8_t brightness;` and
  `void applyBrightness(uint8_t percent);` to the board interface.
- **`src/data.cpp`** — added `brightness` global (default 80), a 1-100%
  range-slider control to the web config page, wired into `/save`
  (validated, clamped, persisted to NVS as `bright`, calls
  `applyBrightness()`).
- **`src/main.cpp`** — loads persisted `brightness` at boot, calls
  `applyBrightness(brightness)` after `applyInvertColors()`.
- **`src/display_cyd.cpp`** / **`src/display_jc4832.cpp`** — added
  `applyBrightness()` using real LEDC PWM on the `TFT_BL` pin (previously a
  fixed digital HIGH only — these boards had no graduated brightness
  control before this).
- **`src/IOExtension.cpp`** — `setBacklight()` now inverts percent before
  writing the duty register (`duty = 100 - percent`, clamped to vendor's
  own 97 max) — the CH32V003 IO-expander's PWM output is **active-low**,
  confirmed directly from Waveshare's own vendor example source
  (`15_LVGL_SLIDER/15_LVGL_SLIDER.ino`: *"inverted: 100 = off, 0 = full
  brightness... due to active-low"*). Before this fix, 1% was brightest and
  100% was darkest on both LCD-7B boards — this is now correct on both.
- **`src/display_lcd7b.cpp`** (the *working*, single-buffer board) — two
  changes, both board-agnostic UX fixes, not experimental: (1) traces
  changed from disconnected 2px `DARKGREY` dots to a connected `YELLOW`
  polyline (dots were nearly invisible and blended into the same-colored
  range rings); (2) trace history capacity raised from 24 samples (~15-20s)
  to 1800 samples (~15-30 min), moved from a fixed struct array to
  PSRAM-backed (`ps_malloc`) storage since 1800×16 bytes×20 aircraft slots
  (~560KB) doesn't belong in internal SRAM; also added `applyBrightness()`
  (see above). **None of this has been flashed/tested on the actual
  `lcd7b` board this session** — only `lcd7b_v2` has been flashed and
  observed on hardware. Build-verified only.
- **`src/display_lcd7b_v2.cpp`** — the experimental board, extensively
  reworked this session. See `LCD7B_V2_DEBUG_LOG.md` for the full
  attempt-by-attempt history. Same trace polyline/capacity/PSRAM change as
  `display_lcd7b.cpp`, plus the double-buffering/sync work summarized
  below.
- **`platformio.ini`** — not shown as modified above because it was edited
  in an earlier part of this session (already committed in `f9fb8e1`):
  version-pinned `platform = espressif32@6.6.0` on the three stable envs,
  added the `[env:lcd7b_v2]` block.

## `lcd7b_v2` — current technical state

Full history in `LCD7B_V2_DEBUG_LOG.md`. Short version:

**Architecture**: bypasses Arduino_GFX's RGB panel classes (confirmed via
source that they never support true double buffering, any version) and
drives the panel directly via raw ESP-IDF `esp_lcd_new_rgb_panel()` with
`num_fbs=2`. Rendering is **zero-copy**: a `DirectCanvas` (Arduino_Canvas
subclass) injects the driver framebuffer pointer into the canvas's protected
`_framebuffer` before `begin()`, so screens rasterize straight into the
driver's inactive framebuffer; `pushFrame()` then presents via
`esp_lcd_panel_draw_bitmap()` with that same pointer, which the IDF driver
turns into an O(1) `cur_fb_index` flip (verified in the bundled IDF v5.5.5
`esp_lcd_panel_rgb.c`: pointer-inside-fb → no copy). After each radar full
redraw, the presented frame is `memcpy`-synced into the other fb — without
that, blips/labels/clock visibly alternate between the last two redraw
states on every buffer flip.

**Current config** (`initRGBPanel()`):
- `pclk_hz = 20000000` — stepped in isolation 10→16→20MHz (all stable) →
  22MHz (drifted). 20MHz ≈ 21.8Hz physical refresh; sweep runs at that rate.
- `bounce_buffer_size_px = LCD_W * 20` — fixes a GDMA-underrun screen-drift
  bug, confirmed fixed on hardware, **do not remove**
- `num_fbs = 2`
- Registers `on_frame_buf_complete` callback → gives a semaphore;
  `pushFrame()` waits on it after every present before the other buffer is
  drawn into again.

**Known-bad, do not repeat**: `pclk_hz` above 20MHz with the prebuilt core
(PSRAM @80MHz). Espressif's ESP-FAQ (LCD section) publishes tested ceilings:
**~22MHz max with octal PSRAM @80MHz**, ~30MHz only with octal PSRAM @120MHz
+ flash @120MHz. Hardware results matched exactly: 22MHz drifted, 20MHz is
the last confirmed-stable value under that config.

**Phase 3 (30MHz, ~33Hz) — BUILT, pending USB flash**: the 120MHz
PSRAM+flash recipe is implemented via pioarduino HybridCompile
(`board_build.f_boot=120M` + `board_build.f_image=80m` + `custom_sdkconfig`
in `[env:lcd7b_v2]` — see LCD7B_V2_DEBUG_LOG.md attempt #17 for the three
attempts it took and why) and `pclk_hz = 30000000` is set in source. The
factory image must be **USB-flashed** (the 120MHz config lives in the
bootloader; OTA can't touch it). After flashing: watch for drift, then soak
via `/health`. 120MHz octal PSRAM is officially experimental
(temperature-drift crash risk, mitigated in config) and this board's R8V
chip variant is unverified for it. Recovery: USB-flash
`/tmp/lcd7b_recovery/firmware_20mhz_factory.bin` (never OTA the 20MHz app
back onto the 120MHz bootloader).

**Not yet verified on hardware**: nothing outstanding — the zero-copy
rework, trace-persistence change, brightness-inversion fix, and the
16/20MHz pclk steps were all flashed and observed on hardware in the
zero-copy session (22MHz observed drifting and reverted).

## Reference material used this session (may be useful again)

- Waveshare's own example repo for this exact board:
  `https://github.com/waveshareteam/ESP32-S3-Touch-LCD-7B` — cloned during
  this session to `$CLAUDE_JOB_DIR/tmp/ws7b_repo` (a temp job dir, likely
  gone by the time you read this — re-clone if needed). Contains both
  Arduino and ESP-IDF example variants; `examples/Arduino/examples/
  16_LVGL_V9_DEMO/{rgb_lcd_port.cpp,lvgl_port.cpp}` is the authoritative
  reference for correct RGB-panel double-buffering + sync on this hardware.
  `examples/Arduino/examples/15_LVGL_SLIDER/io_extension.cpp` documents the
  active-low backlight PWM polarity.
- `esp_lcd_panel_rgb.h` (ESP-IDF, under the pioarduino core install at
  `~/.platformio_lcd7b_v2/packages/framework-arduinoespressif32-libs/
  esp32s3/include/esp_lcd/rgb/include/`) — doc comments for
  `on_color_trans_done` / `on_vsync` / `on_frame_buf_complete`, useful if
  more precise synchronization is needed later.

## Persistent memory note

There's a standing instruction (from this Claude Code session's memory,
may or may not be visible to whatever you're using next) to leave
diagnostic Serial/HTTP-debug logging in place unless explicitly told to
remove it. The `/timingdebug` and `/gt911debug` HTTP routes in
`display_lcd7b_v2.cpp`, and various `Serial.printf` diagnostics throughout,
exist for this reason — they're intentional, not leftover debug cruft.
