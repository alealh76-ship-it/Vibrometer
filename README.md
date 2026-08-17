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
| MOSI (or `DI`) | D11 | fixed |
| MISO (or `DO`) | D12 | fixed |
| SCK  | D13 | fixed |
| CS   | D10 | configurable — `#define SD_CS_PIN` |
| VCC  | 3.3V *or* 5V | see warning below |
| GND  | GND | |

The board confirms the left column at boot, read straight from the variant
macros, so you never have to take this table on faith:

```
[CFG] SPI per the board variant: MOSI=D11 MISO=D12 SCK=D13 SS=D10
```

> **Check the module's own pin labels before wiring — this is the one that
> bites.** Many microSD breakouts print their labels on the **underside only**,
> and the pin *order* is not standardised between module types. Wiring by
> position, from another module, or from a stock photo gives you a silent CMD0
> timeout that looks exactly like a dead card. Pull the module off the
> protoboard and read the silkscreen.
>
> Where a module uses `DI`/`DO`, those names are from the **card's** point of
> view: `DI` → MOSI (D11), `DO` → MISO (D12). Modules labelled `MOSI`/`MISO`
> are already host-relative and connect straight across.

> **3.3 V warning.** The Nano 33 BLE is a 3.3 V board and its GPIO is **not 5 V
> tolerant**. Use a microSD breakout that is 3.3 V-native, or a 5 V module whose
> level shifter also brings **MISO back down to 3.3 V** (many cheap "5V"
> modules only shift the inputs and drive MISO at 5 V, which will slowly damage
> the nRF52840). If in doubt, power the module from the board's 3V3 pin.

> **LED_BUILTIN conflict.** SCK shares a pin with `LED_BUILTIN` on this board,
> so `LED_BUILTIN` is unusable once the SD card is wired. That is why all status
> indication uses the onboard RGB LED.

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
millis,ax,ay,az,dev,ac
12340,-0.0123,0.9876,-0.0456,-0.0021,0.00412
```

> **Format changed after the first study.** Files from the earlier firmware have
> the header `millis,ax,ay,az,magnitude` with an unsigned, `abs()`-folded last
> column. Check the header row before parsing.

- `millis` — `millis()` at sample time, relative to **boot**, not to file start.
  Absolute wall-clock time is not recorded (no RTC); relative time within a
  session is what the analysis needs. Note this means timestamps do **not**
  reset between files, which is convenient for stitching sessions together.
- `ax/ay/az` — g, 4 decimals (`±4 g` range is ~0.000122 g/LSB, so 4 decimals is
  matched to the sensor).
- `dev` — `sqrt(ax²+ay²+az²) − gravityBaseline`, in g. **Signed.** The old
  column took the absolute value, which folded the signal about the baseline and
  destroyed information whenever the baseline was wrong. Signed also means the
  tracked baseline is recoverable offline as `norm − dev`, so you never need the
  serial log to interpret a file.
- `ac` — rolling standard deviation of the norm over `AC_WINDOW_MS`, in g,
  5 decimals. **This is what the trigger runs on.** It has no DC term, so a
  baseline error cannot move it.

`gravityBaseline` is seeded at boot and then tracked continuously, but only while
`ac` says the machine is still, so a wash can never drag it. It is clamped to
0.85–1.15 g. A noisy boot measurement now warns and is still used — a measured
value beats an assumed one, and the tracker corrects it within a few minutes of
quiet.

## Tuning knobs

All at the top of the `.ino`:

| `#define` | Default | What it does |
|---|---|---|
| `SD_CS_PIN` | `10` | SD chip select |
| `SAMPLE_RATE_HZ` | `100` | log rate |
| `VIBE_THRESHOLD` | `0.006f` | g, on the **`ac` metric** — start/continue logging above this |
| `AC_WINDOW_MS` | `1000` | averaging window for the `ac` metric |
| `TRIGGER_CONFIRM_MS` | `500` | must stay above threshold this long to open a file (rejects door bumps) |
| `QUIET_TIMEOUT_MS` | `1500000` (25 min) | quiet time before closing the file |
| `BASELINE_TAU_S` | `300` | baseline tracker time constant (only runs while still) |
| `ACCEL_RANGE_G` | `4` | full scale; confirmed correct by the first study |
| `FLUSH_EVERY_SAMPLES` / `FLUSH_INTERVAL_MS` | `200` / `10000` | buffer push / FAT flush cadence |
| `AUTO_CALIBRATE_BASELINE` | `1` | set `0` to skip the boot seed |

**Every default above that is not a round number was measured**, not guessed —
see "What the first 12-log study changed" below.

**First run:** set `QUIET_TIMEOUT_MS` to something short (say 20 s) and confirm
start/stop works by hand-shaking the machine, then put it back to 25 minutes
before running a real cycle.

## Serial output

Boot diagnostics (IMU/SD init, calibrated baseline, config echo), then a line per
transition, and a stats block on every file close:

```
[>>>] 41233 ms: IDLE -> LOGGING, file LOG0007.CSV, AC level 8.14 mg
[<<<] 3182044 ms: LOGGING -> IDLE (quiet timeout)
      file=LOG0007.CSV duration=3140s samples=313902 peakAC=203.3 mg
      peakDev=0.4412 g baseline=0.9831 g
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
fixed at **D11/D12/D13**, and the boot banner echoes them from the variant
macros so you can confirm rather than assume. Change only `SD_CS_PIN` if your
breakout is wired elsewhere. Two things worth double-checking on the *module*
side: its pin labels (often silkscreened underneath) and the **3.3 V
level-shifting on MISO** — see the warnings above.

**2. Is 100 Hz achievable?** No — it ran at **88.2 Hz**, and the first study
settled both halves of this question with real numbers.

- *SD side — this is where the loss is.* **13.5%** of wall-clock time sat inside
  gaps longer than 15 ms: 22,554 gaps over 50 ms, 45 over 200 ms, worst case
  1,473 ms. My original estimate of ~5% duty was optimistic by roughly 3×. The
  expensive operation is `flush()`, which forces a FAT + directory update, so
  it now runs every 10 s instead of every 1 s while block writes stay frequent.
  Whether that recovers the missing 12 Hz is measurable: watch `effective_rate`
  in the end-of-session line.
- *IMU side.* The stock library fixes the accelerometer ODR at 119 Hz, so a
  100 Hz tick occasionally finds no new data and reuses the previous reading.
  That is the `stale` counter, and it was **0** across all twelve logs — a
  non-issue in practice.

> **Correction.** An earlier version of this file, and the first analysis
> report, suggested dropping to `SAMPLE_RATE_HZ 50` as a fallback and claimed
> "all the energy is below 20 Hz." **That was wrong, and I had not measured it.**
> An FFT of the spin segments shows the fundamental at **13.4 Hz** with a strong
> second harmonic at **26.8 Hz**, and only 50% of the AC energy below 13–17 Hz.
> **37% of the energy sits above 25 Hz.** Sampling at 50 Hz puts Nyquist at
> 25 Hz and folds that 37% back into the band as false low-frequency content —
> it would corrupt exactly the signal the detector depends on. Keep 100 Hz;
> Nyquist then lands at 50 Hz, which is also where the LSM9DS1's own
> anti-aliasing filter rolls off, so the two match.

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
| `R1 = 0x01` | **card and wiring are fine** — go back to the logger |
| all `0xFF` | nothing answering — power, CS, or SCK/MOSI not arriving |
| all `0x00` | MISO held low — miswired, shorted, or module unpowered |
| mixture | partial comms — signal integrity, shorten the wires |

It also prints the core's real `MOSI`/`MISO`/`SCK`/`SS` pin numbers, so you can
confirm the real pin mapping instead of trusting any documentation, and does
a static pull-up/pull-down test on MISO that detects a line being held low —
which SPI traffic alone cannot distinguish from a silent card.

**If the probe reports `R1 = 0x01`**, the card and wiring are fine — go straight
back to `WasherVibeLogger`. The stock `SD` library is only a suspect if
`SD.begin()` still fails *after* the raw probe succeeds, which is rare.

> **Historical note.** During the first debugging round this project suspected
> the SD library and recommended switching to SdFat. That was wrong: the actual
> fault was **the SD module's pins being wired from the wrong map**, because its
> labels are silkscreened on the underside where they are easy to miss. The
> board-side D11/D12/D13 mapping above was correct all along. Stay on the stock
> Arduino `SD` library. If you installed SdFat while chasing this, uninstall it
> or drop back to `SD` 1.2.4 — SdFat v2 also takes over `<SD.h>`, renames
> `Sd2Card`/`SdVolume` to `SdCard`/`FsVolume`, and redefines `F()` as a plain
> string on non-AVR targets, which breaks `SdDiagnostic` and any code typed on
> `const __FlashStringHelper *`.

`WasherVibeLogger` builds against either library — the layer probe compiles out
when the classic classes are absent. `SdDiagnostic` needs the classic `SD`
library and says so with a readable `#error`.

**If layer 1 fails, the most likely cause is power, not signal wiring.** A "5 V"
SD module with an onboard 3.3 V regulator needs **5 V on VCC** (take it from the
Nano's 5V pin, which is live when USB-powered). Fed from 3V3, its regulator
drops below the card's minimum and the card silently never enumerates — which
looks exactly like a wiring fault. A 3.3 V-native module wants 3V3. Check which
one you have before re-checking the jumpers.

The diagnostic also retries at a slower SPI clock. If it only works slow, that's
signal integrity: shorten the jumpers to under ~10 cm. Don't ignore it — it can
init at boot and then corrupt data later at full logging rate.

## What the first 12-log study changed

12 files, 2,064,401 samples, 6.5 h of recording over one 6.9-day uptime. Full
interactive review of every trace: **[Washer Vibration Trace Review](https://claude.ai/code/artifact/56c5b056-3528-4821-ad31-946e26387eaa)**.

| Finding | Evidence | Change |
|---|---|---|
| Baseline never calibrated | fitted device baseline was exactly `1.0000 g`; board rests at `0.9845 g` | seed is now always used; noisy seed warns instead of falling back to 1.0 |
| Baseline drifts | `15.4 mg → 18.7 mg` error over 3.3 days | continuous tracker, `BASELINE_TAU_S`, updates only while still |
| Threshold sat inside the error | 20 mg threshold vs 15–19 mg pedestal; 5 of 12 files contained no cycle | trigger moved to the DC-free `ac` metric at 6 mg |
| Metric barely separated on/off | 1.2× against ground truth; AC metric gives 2.0× | `ac` = rolling σ of the norm |
| 5 min timeout split a cycle | LOG0008–11 separated by 6.3/8.1/8.1 min pauses; worst internal pause 19.5 min | `QUIET_TIMEOUT_MS` → 25 min |
| Scheduler double-sampled | 27,122 samples (1.31%) arrived 0–1 ms after their predecessor | resync advances `missed + 1` intervals |
| SD stalls cost 12 Hz | 13.5% of time in gaps > 15 ms | FAT flush every 10 s instead of 1 s |
| ±4 g correct | peak excursion 1.93 g, zero clipped samples | unchanged, now confirmed |

**Why the `ac` metric is centred before accumulating.** It is a rolling variance,
and the raw norm is ≈1.0 with a variance ≈1e-6 — float32 cannot resolve that
difference (catastrophic cancellation). Storing `norm − baseline` instead puts
the values near zero where float32 has resolution to spare. Verified by replaying
the firmware's exact `acPush()` over all 2.06 M logged samples against a float64
reference: the centred version's worst error is **0.0004 mg**; the naive version
errs by up to **3.6 mg**, which is comparable to the 6 mg threshold and would
have broken the detector outright.

**Still unknown:** whether LOG0008–11 really was one wash. The logger has no
clock, so that is inferred from gap structure, not measured. Noting the start
time of a couple of washes by hand would settle it.

## Known limitations

- No pre-trigger buffer — the first `TRIGGER_CONFIRM_MS` of a cycle's onset is
  not recorded. Easy to add later (small ring buffer flushed at file open) if
  the onset transient turns out to matter.
- Gyro and magnetometer are left enabled by `IMU.begin()` but unused.
- `millis()` rolls over after ~49 days of continuous uptime.
- The gravity baseline is calibrated once at boot, not tracked over time.
