# LCD-7B v2 (`[env:lcd7b_v2]`) — Debug Log

Working log for the double-buffered RGB panel experiment on the Waveshare
ESP32-S3-Touch-LCD-7B. Kept as a separate environment/file
(`platformio.ini` `[env:lcd7b_v2]`, `src/display_lcd7b_v2.cpp`) from the
proven-working `[env:lcd7b]` / `display_lcd7b.cpp`, which is untouched by
any of this and stays available as a fallback.

Goal: a radar sweep that looks "smooth like a watch second hand," without
reintroducing the flicker/tearing/drift the single-buffer version had.

**Rule going forward, because this log exists**: change one variable at a
time, especially anything touching `pclk_hz`, buffer sizing, or
synchronization. Several regressions below came from bundling an unverified
change in with a verified one and losing track of which one caused what.

---

## Timeline

### 1. Single buffer → true double buffer (`num_fbs=2`)
Arduino_GFX's RGB panel classes never support real double buffering at any
version (confirmed by reading the library source directly) — only a single
PSRAM framebuffer. Fix required bypassing Arduino_GFX's RGB bus classes
entirely and driving the panel via raw ESP-IDF `esp_lcd_new_rgb_panel()`
with `num_fbs=2`, which needs ESP-IDF ≥5.1 (arduino-esp32 core 3.x) — hence
the separate `pioarduino` platform / environment.
**Status: kept.**

### 2. Screen drift / horizontal shift ("whole screen shifting left and right")
Root cause: GDMA FIFO underrun reading frame data from PSRAM — Espressif's
own documented failure mode for this driver. Fix: `bounce_buffer_size_px`
(fast internal-SRAM staging buffer DMA reads from instead of PSRAM
directly), set to `LCD_W * 20` (20 lines). **Confirmed fixed on hardware.**
**Status: kept.** This is the value the rest of this log calls "the known-
good bounce buffer."

### 3. pclk tuning, first pass: 30MHz (vendor default) → 10MHz
At 30MHz, *everything* got slower, including pure CPU-side `fillScreen`
(measured, not guessed) — pointed at PSRAM bus contention (CPU-side canvas
drawing and the panel's DMA scan-out both compete for the same PSRAM bus;
a faster pclk gives DMA a bigger continuous share, starving CPU-side
drawing). Dropped to 10MHz, confirmed faster via `/timingdebug`.
**Status: superseded — see attempt 8.** This trade only made sense under
the workload at the time (full-canvas redraw on every tick, not just the
periodic slow one).

### 4. "Two lines, one paused, one moving" ghosting on the sweep
Tried, in order:
- `pushPartial()` (small sub-rect `draw_bitmap`) — blamed for the ghosting
  at the time. **Later found to be innocent — see attempt 6.**
- `refresh_on_demand=1` (stop the panel's continuous autonomous DMA
  re-scan, only trigger explicitly) — caused a **full black screen**. This
  "dumb" RGB panel has no onboard GRAM, unlike a QSPI panel; it needs a
  genuinely continuous signal just to show anything. **Reverted.**
- Diagnosis eventually correctly identified the real cause (see #6).

### 5. Over-claimed hardware limitation
Told the user RGB-parallel panels architecturally can't be as smooth as the
JC4832W535's QSPI panel (which has onboard GRAM). **This was wrong and the
user correctly called it out** — the board supports LVGL smoothly. The real
gap was a naive full-screen-redraw-every-frame implementation vs. LVGL's
default dirty-region-tracking strategy, not a hardware ceiling.

### 6. Root cause of the ghosting, actually found
Traced through the code: the periodic full-redraw path computed the sweep
angle to draw (correct), then called `pushFrame()` — which **blocks for
~85-100ms** — and only *after that returned* recomputed the angle again via
a fresh `millis()` call to store as "previous position" for next tick's
erase. That second computation was ~6-7° further along than what was
actually on screen, so the next fast tick erased a position that had never
been drawn, permanently leaving the real on-screen line un-erased. This
alone produced exactly "one paused line, one moving line."
**Fix: capture the angle once, before the blocking push, reuse it.**
**Status: kept — this was the actual ghosting bug, not `pushPartial()`.**

### 7. Sweep still "stepped" after the ghosting fix
`/timingdebug` showed `pushFrame()` alone (a full 1024×600 push) cost
~75-77ms. The fast-tick sweep update was calling that on every tick, so
every visible step was gated by a 75ms+ transfer regardless of the 30ms
throttle. Re-enabled the already-written-but-shelved `pushPartial()`
(pushes only the small rect around the sweep line) for the fast tick.
**Status: kept.**

### 8. pclk raised 10MHz → 30MHz (Waveshare's own shipped value)
Reasoning at the time: with fast ticks now only pushing a small partial
rect, the CPU/PSRAM-contention trade that justified 10MHz (attempt #3) no
longer applied to the hot path. Also: at 10MHz, the panel's *physical*
refresh rate is capped at ~11Hz (~91ms/frame) purely from pixel-clock math
— a hard ceiling regardless of code efficiency. Waveshare's own LVGL
reference for this exact panel runs at 30MHz with only a **10-line** bounce
buffer (less margin than our 20-line one), so 30MHz looked safe.
**Result: reintroduced the exact horizontal-drift/jumping bug from
attempt #2.** Vendor's reference does far less concurrent CPU-side PSRAM
traffic than this app (WiFi/TLS fetch task, WebServer, full Arduino_GFX
text rendering) — bounce-buffer underrun is a race between DMA drain rate
(set by pclk) and ISR refill rate (slowed by *our* heavier PSRAM
contention), so "safe at 30MHz for their demo" didn't transfer here.
**Status: REVERTED to 10MHz.** Do not raise this again without isolating it
as the *only* change in a build and specifically watching for drift before
touching anything else.

### 9. No hardware synchronization at all (the deeper bug behind #7/#8)
Found by reading Waveshare's own LVGL reference source
(`rgb_lcd_port.cpp`/`lvgl_port.cpp` in `waveshareteam/ESP32-S3-Touch-LCD-7B`
on GitHub) line by line: their flush callback calls `draw_bitmap`, then
**blocks on a frame-completion callback**
(`on_frame_buf_complete`/`on_bounce_frame_finish`) before drawing again.
Our code was firing `draw_bitmap()` with no synchronization at all — just a
1-tick `vTaskDelay`. Nothing stopped us writing into a buffer the DMA
scanner was still reading from. Registered the same callback, added a
binary semaphore, and made both `pushFrame()` and `pushPartial()` wait for
it (draining any stale pending signal first, mirroring vendor's own
`ulTaskNotifyValueClear()` immediately before their wait, so the wait
corresponds to *this* draw call and not a leftover autonomous-refresh
signal from before it). **Status: kept — real, vendor-proven fix,
independent of the pclk value.**

### 10. Traces: dots instead of a line
`fillCircle(px, py, 2, DARKGREY)` at each sample point — isolated 2-3px
dots at typical sample-to-sample spacing don't read as "a path." Also
`DARKGREY` is the *exact same color* as the three range rings, so a dot
landing on or near one visually disappeared. Changed to a connected
polyline (`drawLine` between consecutive samples) in `YELLOW`.
**Status: kept.**

### 11. Traces not lasting
`TRAIL_LEN` was 24 samples (~15-20s of history at this screen's ~1-2Hz
full-redraw sampling rate) in a fixed-size struct member, FIFO-evicted once
full. Raised to 1800 samples (~15-30 min) and moved storage to PSRAM
(`ps_malloc`, allocated once per trail slot and kept across resets) instead
of inline struct arrays, since 1800×16 bytes×20 slots (~560KB) doesn't
belong in internal SRAM. A trail now only clears when its aircraft is
genuinely gone (`TRAIL_STALE_MS`, 15s unseen), not from hitting a small
sample cap. Also fixed a latent bug this change would have caused: the
shift-loop and `count` field used `uint8_t` (max 255), which silently wraps
before reaching 1800 — widened to `uint16_t`.
**Status: kept, not yet verified on hardware.**

### 12. Backlight brightness inverted
Added a brightness slider to the shared web UI (`applyBrightness()`,
one implementation per board). On the LCD-7B boards, initially ported the
percent value straight into the PWM duty register. **Confirmed via
Waveshare's own vendor example source**
(`15_LVGL_SLIDER/15_LVGL_SLIDER.ino`, comment: *"inverted: 100 = off, 0 =
full brightness... due to active-low"*) that the register is active-low —
so higher percent must map to a *lower* duty value. Fixed in
`IOExtension::setBacklight()`: `duty = 100 - percent` before scaling.
**Status: kept, not yet verified on hardware.**

### 13. Zero-copy rendering (the big one)
Read the bundled IDF v5.5.5 `esp_lcd_panel_rgb.c` source directly: when the
`color_data` pointer passed to `esp_lcd_panel_draw_bitmap()` falls INSIDE one
of the driver's own framebuffers, the driver does **no copy at all** — it just
sets `cur_fb_index` to that buffer (adopted at the next bounce-wrap) and
cache-syncs. Exploited this: a `DirectCanvas` subclass injects the driver fb
pointer into `Arduino_Canvas`'s protected `_framebuffer` before `begin()` (so
begin skips its own 1.2MB allocation), the canvas rasterizes straight into
the inactive driver fb, and `pushFrame()` presents with an O(1) index flip.
Deleted the separate 1.2MB canvas, the per-frame full-canvas copy, and the
whole `pushPartial()`/crop-buffer path (a flip *is* the partial update now).
This removed a continuous 1.2MB PSRAM→PSRAM memcpy from the same bus the DMA
scan-out reads — the contention that capped pclk. **Status: kept, verified
on hardware.**

### 14. Buffer-alternation jitter (follow-up bug from #13)
After #13, planes/labels/clock visibly bounced back and forth. Root cause:
the two driver fbs alternate on screen every flip (~90ms), but full redraws
(blips/labels/clock) only ran every 500ms — into ONE buffer — so consecutive
flips alternated between two full-redraw states ~500ms apart. (The old design
never hit this because every push copied the single persistent canvas, so both
fbs always received identical full state.) The sweep line was immune (already
tracked per-buffer). Fix: after each full redraw + present, `memcpy` the
presented frame into the other fb (safe: `waitFrameDone()` guarantees it's
free; reading the mid-scan buffer is fine, only writing it wouldn't be) and
sync its sweep-erase state. Costs one ~40-80ms burst every 500ms, i.e. a
single-frame sweep pause twice a second. **Status: kept, verified on
hardware — planes/labels/clock rock stable.**

### 15. pclk raised in isolation: 16MHz ok → 20MHz ok → 22MHz unstable
With #13's contention removed, stepped pclk as the ONLY change per build,
watching for drift each time. Espressif's ESP-FAQ (LCD section) publishes
tested ceilings: **~22MHz max with octal PSRAM @80MHz** (our prebuilt-core
config — confirmed `CONFIG_SPIRAM_SPEED_80M` in the qio_opi sdkconfig);
**~30MHz needs octal PSRAM @120MHz + flash @120MHz**. Results matched the
FAQ exactly: 16MHz stable, 20MHz stable, **22MHz drifted** (right at the
ceiling, zero margin). Settled on **20MHz ≈ 21.8Hz physical refresh** —
double the old 11Hz ceiling, sweep visibly smooth. Also relevant: the
prebuilt core has 32B data-cache lines and `LCD_RGB_ISR_IRAM_SAFE` off —
Espressif recommends 64B cache lines in bounce mode + IRAM-safe ISR for
high-pclk stability, but both need a custom sdkconfig.
**Status: kept at 20MHz.**

---

## Current state (as of this push)

- `pclk_hz = 20000000` (stepped 10→16→20 ok, 22 unstable; ~21.8Hz physical refresh)
- `bounce_buffer_size_px = LCD_W * 20` (unchanged, known-good)
- `num_fbs = 2`, real double buffering
- **Zero-copy rendering**: canvas draws directly into the driver fb being
  presented; `pushFrame()` is an O(1) buffer-index flip + `on_frame_buf_complete`
  wait. No separate canvas buffer, no per-frame copy, no `pushPartial()`.
- Full redraws (every ~500ms on radar) `memcpy`-sync the presented frame into
  the other fb afterwards, so blips/labels/clock don't alternate between
  buffer flips (#14)
- Sweep fast tick: erase+redraw line per-buffer (positions tracked per fb),
  presents every frame — runs at the panel's physical refresh rate
- Traces: connected YELLOW polyline, PSRAM-backed, up to 1800 samples
  (~15-30 min), only cleared on true staleness
- Brightness: inverted correctly for the active-low PWM register

## Known open / unverified

- Sweep smoothness is now capped by the ~21.8Hz physical refresh at 20MHz
  pclk. Going higher needs the **120MHz-PSRAM recipe** (Waveshare's own heavy
  LVGL demo for this exact board ships it): `custom_sdkconfig` in
  platformio.ini (pioarduino supports it — triggers a full IDF-libs rebuild)
  with `IDF_EXPERIMENTAL_FEATURES`, `SPIRAM_SPEED_120M`, flash QIO 120MHz,
  `ESP32S3_DATA_CACHE_LINE_64B`, `SPIRAM_XIP_FROM_PSRAM`,
  `LCD_RGB_ISR_IRAM_SAFE`, `COMPILER_OPTIMIZATION_PERF`, then pclk 30MHz
  (~33Hz). Caveats: 120MHz octal PSRAM is officially *experimental*
  (documented temperature-drift crash risk); this board's chip is the R8V
  variant, which Espressif has not explicitly confirmed for 120MHz (the
  N16R16V module is explicitly excluded, R8V untested). Not attempted.
- Traces (length + color fix) and brightness (inversion fix) verified on
  hardware as of the zero-copy session.
