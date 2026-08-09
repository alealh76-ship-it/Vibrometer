# Washer Vibration Data Logger

Bench data-collection firmware for an **Arduino Nano 33 BLE Sense (Rev1,
LSM9DS1)** that logs raw accelerometer data from a washing machine to an SD card
as CSV, one file per detected wash cycle.

This is a **study rig**, not the final product. The point is to capture full
cycles — including the mid-cycle pauses that make a naive "vibration stopped =
done" rule fail — so a proper cycle-complete detector can be designed offline in
Python/Excel.

Sketch: [`WasherVibeLogger/WasherVibeLogger.ino`](WasherVibeLogger/WasherVibeLogger.ino)

---

## Hardware / wiring

SPI is fixed on the Nano 33 BLE; only CS is your choice.

| SD module | Nano 33 BLE Sense | Note |
|---|---|---|
| MOSI | D11 | fixed |
| MISO | D12 | fixed |
| SCK  | D13 | fixed |
| CS   | D10 | configurable — `#define SD_CS_PIN` |
| VCC  | 3.3V *or* 5V | see warning below |
| GND  | GND | |

> **3.3 V warning.** The Nano 33 BLE is a 3.3 V board and its GPIO is **not 5 V
> tolerant**. Use a microSD breakout that is 3.3 V-native, or a 5 V module whose
> level shifter also brings **MISO back down to 3.3 V** (many cheap "5V"
> modules only shift the inputs and drive MISO at 5 V, which will slowly damage
> the nRF52840). If in doubt, power the module from the board's 3V3 pin.

> **D13 conflict.** D13 is SCK, so `LED_BUILTIN` is unusable once the SD card is
> wired. That is why all status indication uses the onboard RGB LED.

Power is USB 5 V for now. Card must be FAT16/FAT32 formatted.

## Build

Arduino IDE only — no PlatformIO-specific constructs, no extra build steps.

1. **Boards Manager** → install *Arduino Mbed OS Nano Boards*, select
   **Arduino Nano 33 BLE**.
2. **Library Manager** → install **Arduino_LSM9DS1** and **SD** (by Arduino).
   `SPI` and `Wire` ship with the core.
3. Open `WasherVibeLogger/WasherVibeLogger.ino`, upload, open Serial Monitor at
   **115200**.

## Status LED

| State | LED |
|---|---|
| IDLE — waiting for vibration | short **blue** blink every 3 s |
| LOGGING | **green** blink, 150 ms on per second |
| ERROR — SD init failed / full / write failure | fast **red** blink (retries SD every 5 s) |

An IMU init failure parks the board in the red-blink state permanently — there
is nothing to log without it.

## CSV output

One file per cycle, `LOG0001.CSV`, `LOG0002.CSV`, … The card root is scanned at
session start and numbering continues from the highest existing file, so prior
sessions are never overwritten.

```
millis,ax,ay,az,magnitude
12340,-0.0123,0.9876,-0.0456,0.0234
```

- `millis` — `millis()` at sample time, relative to **boot**, not to file start.
  Absolute wall-clock time is not recorded (no RTC); relative time within a
  session is what the analysis needs. Note this means timestamps do **not**
  reset between files, which is convenient for stitching sessions together.
- `ax/ay/az` — g, 4 decimals (`±4 g` range is ~0.000122 g/LSB, so 4 decimals is
  matched to the sensor).
- `magnitude` — `|sqrt(ax²+ay²+az²) − gravityBaseline|`, in g. This is the raw
  (unsmoothed) trigger metric; the trigger itself runs on a smoothed version of
  it. Fully recomputable offline from the three axes.

`gravityBaseline` is measured at boot over 2 s instead of assuming exactly
1.000 g, so the metric sits at ~0 whatever orientation the board is mounted in.
If the board is moving during that window the calibration is rejected (spread >
0.05 g) and it falls back to 1.0000 g. The value used is printed on boot —
**write it down**, it is not stored in the CSV.

## Tuning knobs

All at the top of the `.ino`:

| `#define` | Default | What it does |
|---|---|---|
| `SD_CS_PIN` | `10` | SD chip select |
| `SAMPLE_RATE_HZ` | `100` | log rate |
| `VIBE_THRESHOLD` | `0.02f` | g, on the **smoothed** magnitude — start/continue logging above this |
| `TRIGGER_CONFIRM_MS` | `500` | must stay above threshold this long to open a file (rejects door bumps) |
| `QUIET_TIMEOUT_MS` | `300000` (5 min) | quiet time before closing the file |
| `VIBE_SMOOTH_SAMPLES` | `25` | EMA length for the trigger signal (~0.25 s @ 100 Hz) |
| `ACCEL_RANGE_G` | `4` | full scale; see below |
| `FLUSH_EVERY_SAMPLES` / `FLUSH_INTERVAL_MS` | `100` / `1000` | buffer push / FAT flush cadence |
| `AUTO_CALIBRATE_BASELINE` | `1` | set `0` to force a 1.0 g baseline |

**First run:** set `QUIET_TIMEOUT_MS` to something short (say 20 s) and confirm
start/stop works by hand-shaking the machine, then put it back to 5 minutes
before running a real cycle.

`VIBE_THRESHOLD` at 0.02 g is a guess for a machine on a solid floor. The
honest way to set it: run one cycle with the threshold low (0.005 g) so it
triggers on almost anything, then look at the recorded `magnitude` during a
genuine pause versus the idle floor and pick a value between them.

## Serial output

Boot diagnostics (IMU/SD init, calibrated baseline, config echo), then a line per
transition, and a stats block on every file close:

```
[>>>] 41233 ms: IDLE -> LOGGING, file LOG0007.CSV, trigger level 0.0241 g
[<<<] 3182044 ms: LOGGING -> IDLE (quiet timeout)
      file=LOG0007.CSV duration=3140s samples=313902 peak=0.4412 g
      dropped=98 stale=0 clipped=0 effective_rate=99.9 Hz
```

The three counters are the ones to watch:

- **dropped** — sample ticks skipped because an SD write overran the budget.
  Timestamps stay honest (the schedule resyncs rather than emitting catch-up
  samples with fake times), so drops show up as gaps in `millis`, not as
  distorted timing.
- **stale** — ticks where the IMU had no new sample and the previous reading was
  reused. See the sample-rate note below.
- **clipped** — samples at/near full scale. Non-zero means raise `ACCEL_RANGE_G`.

---

## Answers to your open questions

**1. SD CS pin / wiring.** Set to **D10** (`SD_CS_PIN`), which is the
conventional choice and clashes with nothing on this board. MOSI/MISO/SCK are
fixed at D11/D12/D13 as in the table above. Change only `SD_CS_PIN` if your
breakout is wired elsewhere. The thing actually worth double-checking is not the
CS pin but the **3.3 V level-shifting on MISO** — see the warning above.

**2. Is 100 Hz achievable?** Yes, with one caveat that is about the IMU, not the
SD card.

- *SD side — fine.* 100 samples/s × ~40 bytes = ~4 kB/s, pushed as one
  ~4 kB block write per second. Even a slow card manages that in well under
  50 ms, against a 1000 ms budget — roughly 5% duty. Sampling is scheduled off
  `micros()` and resyncs after an overrun, so a slow write costs a few dropped
  samples (counted and reported), never a drift in timestamps. Formatting is
  done with integer math as each sample is taken, deliberately avoiding
  `printf("%f")`, which is slow enough at 100 Hz to matter.
- *IMU side — the real limit.* The stock `Arduino_LSM9DS1` library hard-codes
  the accelerometer ODR to **119 Hz** and offers no API to change it. Sampling
  at 100 Hz against a free-running 119 Hz source means occasional ticks find no
  new data; the sketch reuses the previous reading and counts it in **stale**.
  Expect a small non-zero stale count — that is aliasing between the two clocks,
  not a fault. If stale is high or you want exact alignment, the clean fallback
  is `SAMPLE_RATE_HZ 50` (an even sub-multiple region of 119 Hz where every tick
  reliably has fresh data), which is still ~25× the frequency content that
  matters for cycle-state detection. **Recommendation:** keep 100 Hz for the
  first few cycles, check the stale count, drop to 50 Hz if it bothers you.

**3. Full-scale range.** The stock library is hard-wired to **±4 g** with no
public API to change it, so the sketch does two things: it counts **clipped**
samples (any axis at ≥98% of full scale) and reports them at file close, and it
provides an `ACCEL_RANGE_G` override that writes `CTRL_REG6_XL` directly and
rescales the library's output (the library divides raw counts assuming ±4 g, so
the correction factor is applied in `readAccel()`).

±4 g should be plenty — a washer in spin is a sustained vibration on the order
of tenths of a g, not a shock event, and gravity alone eats 1 g of the budget.
If the clip counter comes back non-zero after a spin cycle, set
`ACCEL_RANGE_G` to `8` and re-run. Don't raise it pre-emptively: doubling the
range halves the resolution, and resolution is what you want for characterising
quiet pauses.

## Troubleshooting: "SD init FAILED"

`SD.begin()` returns a single `false` for three unrelated failures, which is
why the first version of this firmware couldn't tell you anything useful. It now
re-runs the layers individually on failure and reports which one broke. Upload
[`SdDiagnostic/SdDiagnostic.ino`](SdDiagnostic/SdDiagnostic.ino) for the same
breakdown with no logger code involved at all.

| Layer that fails | What it means | Where to look |
|---|---|---|
| **1 — SPI / card handshake** | card never responded | power, CS, MOSI/MISO swap, wire length |
| **2 — FAT volume** | card talks fine, **wiring is good** | partition table, exFAT/GPT, card >32 GB |
| **3 — root directory** | volume mounts, directory won't open | corrupt filesystem, reformat |

**If layer 2 fails, stop checking the wiring** — the card answered, so the wiring
is proven. Layer 2 failures are formatting, and given a card that recently had
two partitions, that's the place to look first:

- The library reads **MBR partition slot 1 only.** If your FAT32 volume ended up
  in slot 2, `SD.begin()` fails even though the card mounts fine on a PC. The
  diagnostic prints all four slots so you can see this directly.
- It **cannot read GPT.** Some partition tools default to GPT; a GPT card looks
  formatted on a PC and is invisible here.
- It **cannot read exFAT.** Windows picks exFAT by default for cards >32 GB.
- It supports **SD/SDHC up to 32 GB only** — SDXC (64 GB+) is not supported at
  all, regardless of how it's formatted. The diagnostic prints the capacity.

### Layer 1 keeps failing (errorCode 0x1) after swapping modules and cards

`errorCode 0x1` is `SD_CARD_ERROR_CMD0`: the card never answered the first
command. (`data 0x0` is not meaningful — the CMD0 path doesn't populate
`errorData`, so it reads back its initial value.)

If that survives swapping modules and cards and a continuity check, the fault is
no longer in the module, the card, or the copper. Run
[`SdRawProbe/SdRawProbe.ino`](SdRawProbe/SdRawProbe.ino), which drives SPI by
hand with **no SD library involved** and prints the raw bytes on MISO:

| Probe result | Meaning |
|---|---|
| `R1 = 0x01` | **card works** — the SD library is the fault, switch to SdFat |
| all `0xFF` | nothing answering — power, CS, or SCK/MOSI not arriving |
| all `0x00` | MISO held low — miswired, shorted, or module unpowered |
| mixture | partial comms — signal integrity, shorten the wires |

It also prints the core's real `MOSI`/`MISO`/`SCK`/`SS` pin numbers, so you can
confirm the D11/D12/D13 mapping instead of trusting the documentation, and does
a static pull-up/pull-down test on MISO that detects a line being held low —
which SPI traffic alone cannot distinguish from a silent card.

**If the probe reports `R1 = 0x01`**, the stock `SD` library is the problem, and
switching to **SdFat** (Bill Greiman, v2) is the fix.

> **Do not paste SdFat into the diagnostic sketches.** `SdDiagnostic` exists to
> test the stock SD library's layers and is built on `Sd2Card`/`SdVolume`, which
> SdFat v2 does not have. `SdRawProbe` uses no SD library at all, by design.
> Both now `#error` with a readable message rather than emitting a wall of
> confusing type errors. The SdFat change applies to
> `WasherVibeLogger.ino` **only**, and it is a small port, not a paste-in:
>
> - `#include <SdFat.h>` replaces `#include <SD.h>`, plus `SdFat SD;`
> - `SD.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4)))` replaces
>   `SD.begin(SD_CS_PIN)` — inside `initSd()`, not at file scope
> - `reportSdFailure()` must be dropped or rewritten: its `Sd2Card`/`SdVolume`
>   layer probe has no SdFat v2 equivalent
>
> Everything else the logger calls (`open`, `exists`, `write`, `flush`,
> `close`, `openNextFile`) is API-compatible.

Note that installing SdFat alongside SD also **redefines `F()`** as a plain
string on non-AVR targets. Any code that types a variable or parameter as
`const __FlashStringHelper *` stops compiling as soon as SdFat is on the
include path — worth knowing if you add debug helpers of your own.

**If layer 1 fails, the most likely cause is power, not signal wiring.** A "5 V"
SD module with an onboard 3.3 V regulator needs **5 V on VCC** (take it from the
Nano's 5V pin, which is live when USB-powered). Fed from 3V3, its regulator
drops below the card's minimum and the card silently never enumerates — which
looks exactly like a wiring fault. A 3.3 V-native module wants 3V3. Check which
one you have before re-checking the jumpers.

The diagnostic also retries at a slower SPI clock. If it only works slow, that's
signal integrity: shorten the jumpers to under ~10 cm. Don't ignore it — it can
init at boot and then corrupt data later at full logging rate.

## Known limitations

- No pre-trigger buffer — the first `TRIGGER_CONFIRM_MS` of a cycle's onset is
  not recorded. Easy to add later (small ring buffer flushed at file open) if
  the onset transient turns out to matter.
- Gyro and magnetometer are left enabled by `IMU.begin()` but unused.
- `millis()` rolls over after ~49 days of continuous uptime.
- The gravity baseline is calibrated once at boot, not tracked over time.
