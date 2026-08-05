# Handoff — CYD28 Plane Spotter

Written for whoever/whatever picks this up next. Facts and current state
only, organized so you don't have to re-derive anything from git history or
chat logs.

## ⛔ READ FIRST — device is mid-recovery (as of 2026-08-05)

**The LCD-7B panel is boot-looping and needs a USB flash to recover.** Full
story:

- A bad OTA (120MHz-config app on the stock 80MHz bootloader — the
  `lcd7b_v2`/`lcd7b_v3` mixup) started the instability; several later OTA
  uploads then **died mid-write** because flash sector erases stall the CPU
  cache longer than the task-watchdog timeout (reset_reason 6). Those dead
  uploads left **garbage in the OTA slot at 0x650000**, and the bootloader
  keeps selecting it → "invalid magic byte" → reset loop (confirmed from the
  serial log).
- The new build (`features` branch, `abbeff1` + later) fixes the OTA path
  (TWDT tolerant during upload, fetch task suspended during flash) — but it
  can't get on the device OTA, because OTA is what's broken.

**Recovery = USB flash into BOTH OTA slots.** The UART port (not the one
labeled USB) enumerates as `/dev/cu.usbmodem5B910516231` (a CH343 bridge).
DTR/RTS auto-reset is NOT wired on this board — download mode is manual only:

1. Unplug cable. Hold **BOOT**. Plug in. Count 3. Release. Screen black
   forever = download mode. (If the radar appears, BOOT didn't engage.)
2. Then run:
   ```
   cd /Users/luketrav/Projects/flight_radar
   ~/.platformio_lcd7b_v2/penv/bin/python ~/.platformio_lcd7b_v2/packages/tool-esptoolpy/esptool.py \
     --chip esp32s3 --port /dev/cu.usbmodem5B910516231 --before no_reset --after no_reset --baud 115200 \
     write_flash 0x0 .pio/build/lcd7b_v2/firmware.factory.bin 0x650000 .pio/build/lcd7b_v2/firmware.bin
   ```
   Writing both slots matters: otadata may point at either, and both have
   held garbage at some point. If the .pio build dir is gone (cleaned),
   rebuild first: `PLATFORMIO_CORE_DIR=~/.platformio_lcd7b_v2 pio run -e lcd7b_v2`.
3. One attempt DID connect and write 13% before the chip's crash-reset
   killed it — the path works; it's a persistence/timing game. Best results
   came from flashing immediately after a fresh BOOT-held power-up.
   Marginal USB power is a suspected factor (panel + backlight + flash
   writes) — a high-current port / powered hub helps.

After a successful flash: the device should boot the `features`-branch
build, answer `http://192.168.1.75/health`, and OTA should work again
(verify with one no-op OTA re-upload). If it still boot-loops, check
`/health`'s `prev_boot_phase` field (crash forensics — the loop-phase
marker from the previous boot, in RTC RAM).

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

Branch `features` (pushed) is ~25 commits ahead of `master` and is what the
device last ran. Master is tagged `known-good-2026-08-03` (the stable
pre-feature state). Unmerged work on `features`:

- **Display engine**: zero-copy double buffering at 20MHz pclk; staged
  (sliced) full redraws into ping-pong PSRAM staging buffers with an atomic
  swap; vsync-aligned direct writes during the copy phase; phosphor
  afterglow (configurable); background-restore sweep erase (no fuzzy
  labels); plotLine/restoreLine identical-rasterizer pair.
- **Features**: WiFi multi-network + fallback setup AP; map location picker
  (Leaflet/OSM via CDN, guarded so a blocked CDN can't break Save);
  traffic filter/watchlist (5 rules: callsign/reg/type prefix ×
  highlight/hide/only/alert + quiet mode + banner); QR code + URL on stats
  screen; animated weather widget + day/night auto-dim; auto-cycle (kiosk
  mode, default 5s); vector map underlay (Natural Earth coastlines +
  borders, polar-cached); WiFi scan picker; password masking; /reboot
  button; /health diagnostics (heap, fetch stage, touch log, staging
  state, prev_boot_phase crash forensics).
- **Fixes**: deferred NVS persistence everywhere (flash writes were
  glitching the panel / watchdogging mid-save); staging slice starvation
  (content refreshed every ~20s — was the "planes frozen/zoom dead"
  report); trail polar cache; PSRAM JSON docs + payload via writeToStream;
  httpsMutex (one outbound TLS at a time); nightly-reboot option.
- **Rejected/reverted with evidence**: 30-line bounce buffer (heap
  starvation), esp_async_memcpy (bus-bound), SIMD on PSRAM (no gain),
  esp_lcd_rgb_panel_restart after config writes (suspect in the boot
  loop — reverted), 22MHz+ pclk on 80MHz PSRAM (drift).
- **Pending**: `lcd7b_v3` (120MHz PSRAM + 64B cache line + IRAM-safe ISR +
  30MHz pclk) is built and unflashed — the USB flash recovery above is the
  same procedure it needs. Merge `features` → master after the device is
  verified stable post-recovery.

DISPLAY_SMOOTHNESS.md consolidates the display-engine learnings.
LCD7B_V2_DEBUG_LOG.md has the chronological attempt history (#1-19).
The regression checklist is below.

## What changed in each file (older session notes)

Historical note — the section below was written mid-way through the v2
bring-up; the features branch has since changed far more than it lists. It
is kept for context but the git log on `features` is the accurate record.

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

## Regression checklist (run after any feature batch, before tagging known-good)

Live checks against the device (replace IP):
1. `/health` — no reset_reason change you didn't cause; heap_min_free and
   heap_largest_block stable vs. pre-change baseline; api_ok incrementing,
   api_fail flat; fetch_stage=5; last_cycle_age_ms ~500 on radar screens.
2. **Persistence**: set 3+ distinctive values across different settings
   groups via /save, then POST /reboot, verify they read back correctly.
   (Covers configMaintain flush + the /reboot pre-flush.)
3. **Save-under-load**: 10 rapid /save calls (range 30↔120) — all 200, no
   reset, screen keeps rendering (user visually confirms no glitch/desync).
4. **Screen cycle**: confirm screens advance on the auto-cycle interval
   (`screen` field in /health changes), and that touching the panel pauses
   cycling for one interval.
5. **Zoom buttons**: 10 rapid +/- presses on each radar screen — range
   changes visibly within ~1s, no display glitch.
6. **Sweep quality** (user eyes): no jitter, no splits, no ghost lines,
   labels stay sharp for 30+ min, afterglow fades evenly.
7. **All four envs build**: `pio run -e cyd -e jc4832 -e lcd7b` plus
   `PLATFORMIO_CORE_DIR=~/.platformio_lcd7b_v2 pio run -e lcd7b_v2` — in
   that order LAST for v2 (pio auto-cleans .pio/build between core dirs;
   always flash v2 immediately after building it).
8. **Soak**: leave running 1h+; cron `/health` watch for heap stair-step
   or unexpected resets.

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
