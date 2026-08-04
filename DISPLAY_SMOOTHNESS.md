# How the LCD-7B v2 Display Engine Achieves a Smooth Radar

Everything we learned getting a 1024×600 "dumb" RGB panel to run a tear-free,
jitter-free radar sweep on an ESP32-S3. This is the consolidated version;
`LCD7B_V2_DEBUG_LOG.md` has the chronological attempt-by-attempt history.

## The hardware reality (know your wall)

- The ST7701 panel has **no controller and no GRAM**. The ESP32-S3's LCD_CAM
  peripheral must continuously stream all 614,400 pixels from PSRAM via GDMA,
  every frame, forever. Stop streaming and the screen goes black.
- **Physical refresh rate = pclk_hz / total-pixel-clocks-per-frame**
  (1024×600 + porches ≈ 916k) — a hard ceiling no code can beat. At 10MHz
  that's ~11Hz; at 20MHz ~21.8Hz; at 30MHz ~33Hz.
- **Octal PSRAM @80MHz ≈ 160MB/s theoretical, ~125MB/s practical read,
  ~30MB/s CPU write — shared** between the LCD scan-out (pclk × 2 bytes),
  the CPU rasterizer, and flash (same MSPI bus). The display stream alone
  eats 40MB/s at 20MHz pclk.
- Espressif's tested PCLK ceilings (ESP-FAQ): ~11MHz quad-PSRAM @80M,
  **~22MHz octal @80M**, **~30MHz octal @120M** (120M also needs flash @120M,
  64B data-cache lines, and is officially experimental — temperature-sensitive).
- Raising pclk past the ceiling causes **GDMA bounce-buffer underrun**, which
  shows up as horizontal line drift. The bounce buffer (internal-SRAM staging,
  `bounce_buffer_size_px`) exists to absorb this; its refill is a CPU memcpy
  in the GDMA EOF ISR (~26MB/s of hidden CPU at 20MHz).

## The pipeline (what runs every frame)

1. **Two driver-owned framebuffers** (`num_fbs=2`, ESP-IDF
   `esp_lcd_new_rgb_panel`, bypassing Arduino_GFX's single-buffered RGB
   classes entirely). A `DirectCanvas` subclass injects the driver's fb
   pointer into `Arduino_Canvas`'s protected `_framebuffer` before `begin()`,
   so all screens rasterize straight into the inactive panel fb.
2. **Zero-copy present**: `esp_lcd_panel_draw_bitmap()` with a pointer that
   lies *inside* a driver fb is an O(1) `cur_fb_index` flip — no 1.2MB copy.
   Verified in the IDF v5.5.5 `esp_lcd_panel_rgb.c` source.
3. **Hardware sync**: every present waits on the `on_frame_buf_complete`
   semaphore before the other buffer is touched again. Never write into a
   buffer the DMA scanner is reading.
4. **Per-buffer sweep state**: the two fbs alternate on every present, so
   each buffer remembers its own sweep-line position for erasing.
5. **Staged (scheduled) full redraws**: the next frame renders into a
   ping-pong pair of PSRAM staging canvases in ~30ms slices between sweep
   ticks (fill / furniture+static / trails×2 / blips), is band-copied into
   the inactive fb, swapped with ONE atomic present, then mirrored into the
   other fb. No 200-300ms blocking burst; the sweep never freezes.
6. **Phase-correct sweep updates**:
   - IDLE + render phases: zero-copy tick (erase+draw in inactive fb, O(1)
     present) — never visible mid-scan.
   - Copy/swap/sync phase (fbs temporarily differ, presents would alternate
     content): direct write into the ACTIVE fb, but **only right after
     VSYNC** (vblank ~4ms), so the scan never catches a half-updated line.
7. **Erase = restore, not black**: the sweep erase restores pixels from the
   last completed staged frame (`bgFrame`, which never has a sweep line baked
   in). Erasing to BLACK cut gashes through labels/trails. Draw and erase use
   the *identical* Bresenham (`plotLine`/`restoreLine`) — mismatched
   rasterizers leave specks.

## The CPU budget (algorithmic wins, because the bus is the wall)

- **Core split**: rendering/touch on Core 1; data fetch AND the web server
  on Core 0. HTTP never steals render time. (I2C gets a mutex: backlight
  writes from web handlers vs touch reads.)
- **Trail polar cache**: distance + bearing sin/cos per trail sample computed
  ONCE at append, not every sample every 500ms (was ~200ms/frame at full
  trails). Recomputed only if home location changes.
- **Raw memset fills** instead of canvas fillScreen (~65ms vs ~74ms; write-
  bound either way).
- **No per-frame big copies beyond the staged band syncs** — and those are
  sliced so they never block the sweep.

## What did NOT work / was rejected (with evidence)

- **pushPartial dirty-rect pushes** — the ghosting blamed on them was
  actually a stale-angle bug; the zero-copy flip made the whole concept
  obsolete anyway.
- **refresh_on_demand=1** — black screen; the panel needs a continuous signal.
- **30MHz pclk on 80MHz PSRAM** — drift; above Espressif's tested ceiling.
- **esp_async_memcpy (GDMA M2M)** — supports PSRAM, but per-transfer overhead
  (descriptor alloc + cache msync) makes it a wash at our band sizes;
  PSRAM→PSRAM is bus-bound (~26MB/s) regardless.
- **SIMD (dsps_*_aes3, 128-bit TIE)** — 4-5× on internal RAM, zero gain on
  PSRAM (bus-limited). Useful only for internal-RAM buffers.
- **XIP-from-PSRAM + 30MHz scan-out in a mixed config** — starved CPU
  instruction fetches (drawing got 20-50× slower).
- **`# CONFIG_X is not set` lines in platformio.ini custom_sdkconfig** —
  silently stripped by the INI parser; choice-symbol overrides need the
  `board_build.f_boot` route (see LCD7B_V2_DEBUG_LOG.md #17).

## The real hardware unlocks (phase 3, `env:lcd7b_v3`)

- **64B data-cache lines**: +49% PSRAM sequential read throughput (measured,
  Qiita benchmarks) and required by Espressif for bounce-mode stability.
- **120MHz octal PSRAM + 120MHz flash**: 1.5× bus, enabling 30MHz pclk
  (~33Hz refresh). Experimental feature; needs USB flash (bootloader carries
  the config). Temperature-drift risk mitigated with
  `SPIRAM_TIMING_TUNING_POINT_VIA_TEMPERATURE_SENSOR`.
- Beyond that: **ESP32-P4 + MIPI-DSI panel** (60Hz, PPA 2D accelerator) is
  the genuine next tier — different chip, no pixel-clock wall.
