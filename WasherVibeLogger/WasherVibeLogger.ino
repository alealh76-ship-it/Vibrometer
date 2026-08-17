/*
 * WasherVibeLogger — raw vibration data logger for washing-machine cycle study
 * ---------------------------------------------------------------------------
 * Board : Arduino Nano 33 BLE Sense (Rev1, LSM9DS1 IMU)
 *         Boards Manager -> "Arduino Mbed OS Nano Boards"
 * Libs  : Arduino_LSM9DS1, SD, SPI   (all via Library Manager / bundled)
 *
 * Purpose
 *   Bench data-collection tool, NOT production firmware. It records raw
 *   accelerometer data at a fixed rate to CSV on an SD card for every detected
 *   wash cycle, so the "cycle complete" detector can be designed offline in
 *   Python/Excel. Washers pause mid-cycle, so the stop rule here is
 *   deliberately lazy: we keep logging through pauses and only close the file
 *   after a long quiet period, capturing an idle tail rather than truncating.
 *
 * State machine
 *   IDLE    -> sampling, not writing. LED: brief blue blink every 3 s.
 *   LOGGING -> sampling + buffering + periodic SD flush. LED: green blink.
 *   ERROR   -> SD missing / init failed / write failed. LED: fast red blink.
 *              Retries SD init every SD_RETRY_INTERVAL_MS.
 *
 * CSV format (one file per cycle, LOG0001.CSV, LOG0002.CSV, ...)
 *   millis,ax,ay,az,dev,ac
 *   - millis   : millis() at sample time, relative to boot (no RTC)
 *   - ax/ay/az : g, 4 decimals (LSM9DS1 @ +/-4 g is ~0.000122 g/LSB)
 *   - dev      : sqrt(ax^2+ay^2+az^2) - gravityBaseline, in g. SIGNED, unlike
 *                the old `magnitude` column: taking the absolute value folded
 *                the signal about the baseline and destroyed information when
 *                the baseline was wrong. Signed also makes the tracked
 *                baseline recoverable offline as (norm - dev).
 *   - ac       : rolling standard deviation of the norm over AC_WINDOW_MS, in
 *                g, 5 decimals. THIS is what the trigger runs on. It carries
 *                no DC term, so a baseline error cannot move it.
 *
 * Format changed after the first 12-log study. Files written by the earlier
 * firmware have the header `millis,ax,ay,az,magnitude` and an unsigned,
 * abs()-folded last column; check the header row before parsing.
 *
 * Wiring (SPI pins are fixed on the Nano 33 BLE; only CS is your choice)
 *   SD MOSI -> D11    SD MISO -> D12    SD SCK -> D13    SD CS -> D10
 *   The board also prints this mapping at boot, straight from the variant
 *   macros, so you can confirm it rather than trust this comment.
 *
 *   CHECK THE MODULE'S OWN PIN LABELS FIRST. Many microSD breakouts print
 *   them on the UNDERSIDE only, and the pin ORDER differs between module
 *   types — do not infer it from another module or a stock photo. Note also
 *   that DI/DO, where used, are named from the card's point of view:
 *   DI -> MOSI (D11), DO -> MISO (D12).
 *   SD VCC  -> see README: the Nano 33 BLE is a 3.3 V board and is NOT 5 V
 *              tolerant. Use a 3.3 V-native microSD breakout, or a 5 V module
 *              whose level shifter also shifts MISO back down to 3.3 V.
 *   NOTE: SCK shares a pin with LED_BUILTIN on this board, so LED_BUILTIN is
 *         unusable while the SD card is wired. That is why all status
 *         indication uses the onboard RGB LED.
 */

#include <Arduino_LSM9DS1.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <strings.h>   /* strcasecmp / strncasecmp */

/* The layer-by-layer failure probe further down uses Sd2Card/SdVolume, which
 * exist in the classic Arduino SD library (1.2.x). SdFat v2 — and SD 1.3.x,
 * which is built on top of it — renamed them to SdCard/FsVolume. Detect which
 * one we got and degrade gracefully rather than refusing to build: the logger
 * itself only needs begin/open/write/flush/close, and must never fail to
 * compile over a diagnostic convenience. */
#if defined(SD_CARD_TYPE_SD1)
#define HAVE_SD_LAYER_PROBE 1
#else
#define HAVE_SD_LAYER_PROBE 0
#endif

/* ==========================================================================
 * TUNING CONSTANTS — the knobs you will actually touch during testing
 * ========================================================================== */

/* Chip-select pin for the SD module. MOSI/MISO/SCK are fixed by the board. */
#define SD_CS_PIN               10

/* Sample rate for the CSV log, in Hz.
 * NOTE: the stock Arduino_LSM9DS1 library runs the accelerometer at a fixed
 * 119 Hz ODR, so anything above ~100 Hz will mostly duplicate samples. See the
 * "stale" counter printed at end of session: it counts ticks where the IMU had
 * no new data and the previous reading was reused. */
#define SAMPLE_RATE_HZ          100

/* Activity threshold, in g, applied to the AC metric (`ac` column).
 * Measured from the first 12-log study: with the machine confirmed off the AC
 * metric sits at 0.4-1.3 mg and its 99th percentile is 4.5 mg; a running
 * machine is 6-200 mg. A sweep over that data puts the knee at 5-6 mg, where
 * false triggers are 0.33% of idle seconds and 52% of cycle time is captured.
 * Below 4 mg false triggers climb sharply; above 12 mg gentle agitation starts
 * being discarded. */
#define VIBE_THRESHOLD          0.006f

/* Averaging window for the AC metric. 1 s is long enough to average out the
 * sensor noise floor and short enough to catch the start of agitation. */
#define AC_WINDOW_MS            1000

/* How long the AC metric must stay above threshold before we open a file.
 * Rejects door bumps, someone leaning on the machine, etc. */
#define TRIGGER_CONFIRM_MS      500UL

/* How long the machine must stay quiet before we close the file and go IDLE.
 * The first study ran this at 5 minutes and it CUT ONE WASH INTO FOUR FILES
 * (LOG0008-11, separated by real pauses of 6.3, 8.1 and 8.1 minutes). Measured
 * over 537 mid-cycle pauses: median 2 s, 90% inside 37 s, but 11 outlast
 * 5 minutes, 2 outlast 15, and the longest is 19.5. The distribution only
 * reaches zero at 20 minutes, so 25 is the smallest defensible setting and
 * leaves margin for a wash that study never captured. Disk cost of the extra
 * idle tail is ~6 MB per cycle. */
#define QUIET_TIMEOUT_MS        (25UL * 60UL * 1000UL)  /* 25 minutes */

/* Time constant of the gravity-baseline tracker, in seconds. The baseline only
 * moves while the AC metric says the machine is still, so this never chases a
 * wash cycle. It exists because the resting norm DRIFTS: over the 3.3 days of
 * the first study it moved 15.4 -> 18.7 mg away from 1 g, most likely with
 * temperature. A one-shot calibration at boot cannot track that. */
#define BASELINE_TAU_S          300.0f

/* Hard limits on the tracked baseline, so one wild transient cannot poison it.
 * The board's true resting norm was 0.9813-0.9846 g across the whole study. */
#define BASELINE_MIN_G          0.85f
#define BASELINE_MAX_G          1.15f

/* Accelerometer full-scale range, in g. The stock library is hard-wired to
 * +/-4 g; any other value here is applied by writing CTRL_REG6_XL directly and
 * rescaling the library's readings (see applyAccelRange()). Valid: 2, 4, 8, 16.
 * Leave at 4 unless the clip counter reports clipping during spin. */
#define ACCEL_RANGE_G           4

/* Log buffer. Samples are formatted into this buffer as they are taken and
 * pushed to the SD card in one block, instead of writing per sample.
 *
 * The first study logged at an effective 88.2 Hz against a 100 Hz target:
 * 13.5% of wall-clock time sat inside gaps over 15 ms, caused by SD writes.
 * The expensive part is flush(), which forces a FAT + directory update, so the
 * FAT flush is now every 10 s instead of every 1 s while the block writes stay
 * frequent. Cost of a power cut is up to 10 s of samples instead of 1 s, which
 * is an easy trade on a USB-powered bench rig. Watch effective_rate in the
 * end-of-session line to see whether this actually bought anything. */
#define LOG_BUFFER_BYTES        8192
#define FLUSH_EVERY_SAMPLES     200     /* push buffer to file every N samples */
#define FLUSH_INTERVAL_MS       10000UL /* ...and force a FAT flush this often */

/* Boot-time seed for the gravity baseline. This only has to be roughly right —
 * the continuous tracker above corrects it — but it must never be a hard-coded
 * 1.0 g. In the first study the spread check rejected the measurement (almost
 * certainly because the board was being handled at power-on, which is exactly
 * when someone is touching it) and the fallback left a 15-19 mg error sitting
 * underneath a 20 mg trigger threshold. A noisy measurement now WARNS and is
 * still used, because a measured value is always closer than an assumed one. */
#define AUTO_CALIBRATE_BASELINE 1
#define CAL_SAMPLES             200     /* 2 s @ 100 Hz */
#define CAL_MAX_SPREAD_G        0.05f   /* only controls the warning now */

/* Optional DS3231 real-time clock on the EXTERNAL I2C bus (A4/A5 = `Wire`).
 * The onboard IMU lives on the internal bus (`Wire1`), so the two never meet.
 * Set to 1 once the part is fitted; everything below compiles either way.
 *
 * Talked to by register, not through a library: the DS3231's register map is
 * tiny and fixed, and that removes a Library Manager dependency along with any
 * chance of an API mismatch. Register 0x0F bit 7 is the oscillator-stop flag,
 * which is how we know whether the stored time is trustworthy at all. */
#define USE_DS3231              0
#define DS3231_ADDR             0x68

/* How long the machine must be continuously quiet before a provisional
 * baseline is snapped to the real resting norm. Mid-cycle pauses count — the
 * machine is genuinely still then, so the reading is genuinely valid. */
#define BASELINE_SNAP_QUIET_MS  5000UL

/* Misc */
#define SERIAL_BAUD             115200
#define SERIAL_WAIT_MS          3000UL  /* don't block forever without USB */
#define SD_RETRY_INTERVAL_MS    5000UL
#define MAX_LOG_INDEX           9999

/* ==========================================================================
 * Internals
 * ========================================================================== */

enum State { ST_IDLE, ST_LOGGING, ST_ERROR };

static State    state          = ST_IDLE;
static File     logFile;
static char     logName[16]    = {0};

/* Sample scheduling (micros-based so it doesn't drift with loop jitter) */
static const uint32_t sampleIntervalUs = 1000000UL / SAMPLE_RATE_HZ;
static uint32_t nextSampleUs   = 0;

/* Latest accelerometer reading, reused if the IMU has nothing new this tick */
static float    lastAx = 0.0f, lastAy = 0.0f, lastAz = 1.0f;

/* Detection signals */
static float    gravityBaseline = 1.0f;   /* g, seeded at boot then tracked */
static float    acRms           = 0.0f;   /* rolling std of the norm, g */
static uint32_t aboveSinceMs    = 0;      /* 0 = currently below threshold */
static uint32_t lastActiveMs    = 0;      /* last time we were above threshold */
static bool     baselineProvisional = true;  /* seed not yet trusted */
static uint32_t quietSinceMs    = 0;      /* 0 = not currently quiet */

/* Sliding window for the AC metric. Deviations from the baseline are stored,
 * not raw norms: the raw norm is ~1.0 and its variance is ~1e-6, which a
 * sum-of-squares in float32 cannot resolve (catastrophic cancellation).
 * Centring first puts the stored values near zero, where float32 has plenty
 * of resolution to spare. */
#define AC_WINDOW  ((uint16_t)((uint32_t)SAMPLE_RATE_HZ * AC_WINDOW_MS / 1000UL))
static float    acBuf[AC_WINDOW];
static uint16_t acHead  = 0;
static uint16_t acCount = 0;
static float    acSum   = 0.0f;
static float    acSumSq = 0.0f;
static const float baselineAlpha = 1.0f / (BASELINE_TAU_S * (float)SAMPLE_RATE_HZ);

/* Accel scale correction, applied when ACCEL_RANGE_G != 4 (library assumes 4) */
static float    accelScale      = 1.0f;
static float    clipThresholdG  = (float)ACCEL_RANGE_G * 0.98f;

/* Write buffer */
static char     logBuf[LOG_BUFFER_BYTES];
static size_t   logBufLen      = 0;
#define LINE_MAX 80     /* worst-case formatted line length, incl. newline */

/* Per-session statistics, printed on close */
static uint32_t sessionStartMs = 0;
static uint32_t nSamples       = 0;
static uint32_t nSinceWrite    = 0;
static uint32_t lastFlushMs    = 0;
static uint32_t nStale         = 0;   /* IMU had no new data at tick time */
static uint32_t nDropped       = 0;   /* ticks skipped (SD write overran) */
static uint32_t nClipped       = 0;   /* samples at/over full scale */
static float    sessionPeakAc  = 0.0f;   /* g, peak of the AC metric */
static float    sessionPeakDev = 0.0f;   /* g, peak |norm - baseline| */

static uint32_t lastSdRetryMs  = 0;

/* Serial command line (see handleCommand) */
static char     cmdBuf[40];
static uint8_t  cmdLen = 0;

#if USE_DS3231
static bool     rtcOk = false;            /* RTC present AND time trustworthy */
#endif

/* ==========================================================================
 * Onboard RGB LED (pins 22/23/24 on the Nano 33 BLE — active LOW)
 * ========================================================================== */

static void ledInit() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH);   /* HIGH = off */
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

static void ledSet(bool r, bool g, bool b) {
  digitalWrite(LEDR, r ? LOW : HIGH);
  digitalWrite(LEDG, g ? LOW : HIGH);
  digitalWrite(LEDB, b ? LOW : HIGH);
}

/* Non-blocking status blink, driven straight off millis(). */
static void ledUpdate() {
  const uint32_t t = millis();
  switch (state) {
    case ST_IDLE:     ledSet(false, false, (t % 3000UL) < 60UL);  break;
    case ST_LOGGING:  ledSet(false, (t % 1000UL) < 150UL, false); break;
    case ST_ERROR:    ledSet((t % 400UL) < 200UL, false, false);  break;
  }
}

/* ==========================================================================
 * Fast, dependency-free number formatting
 *
 * snprintf("%f") pulls in the floating-point printf machinery and is slow
 * enough to matter at 100 Hz, so numbers are formatted with integer math.
 * Each helper returns the number of characters written (no NUL terminator).
 * ========================================================================== */

static size_t appendUInt32(char *dst, uint32_t v) {
  char tmp[10];
  uint8_t n = 0;
  if (v == 0) {
    dst[0] = '0';
    return 1;
  }
  while (v > 0 && n < sizeof(tmp)) {
    tmp[n++] = (char)('0' + (v % 10));
    v /= 10;
  }
  for (uint8_t i = 0; i < n; i++) dst[i] = tmp[n - 1 - i];
  return n;
}

/* Fixed-point rendering of `v` with `decimals` digits, e.g. -0.0123 */
static size_t appendFixed(char *dst, float v, uint8_t decimals) {
  if (isnan(v) || isinf(v)) {              /* keep the CSV parseable */
    memcpy(dst, "nan", 3);
    return 3;
  }

  uint32_t scale = 1;
  for (uint8_t i = 0; i < decimals; i++) scale *= 10;

  size_t  n      = 0;
  int32_t scaled = (int32_t)lroundf(v * (float)scale);
  if (scaled < 0) {
    dst[n++] = '-';
    scaled = -scaled;
  }

  n += appendUInt32(dst + n, (uint32_t)(scaled / (int32_t)scale));
  dst[n++] = '.';

  uint32_t frac = (uint32_t)(scaled % (int32_t)scale);
  for (uint8_t i = decimals; i > 0; i--) {  /* zero-padded fraction */
    uint32_t div = 1;
    for (uint8_t j = 1; j < i; j++) div *= 10;
    dst[n++] = (char)('0' + ((frac / div) % 10));
  }
  return n;
}

/* ==========================================================================
 * IMU
 * ========================================================================== */

/* The stock Arduino_LSM9DS1 library hard-codes CTRL_REG6_XL = 0x70
 * (119 Hz ODR, +/-4 g) and divides raw counts by 32768/4 in readAcceleration().
 * If a wider range is requested we rewrite that register ourselves and scale
 * the library's output back to true g. Only touched when ACCEL_RANGE_G != 4. */
static bool applyAccelRange() {
#if ACCEL_RANGE_G == 4
  accelScale = 1.0f;
  return true;
#else
  uint8_t reg;
  switch (ACCEL_RANGE_G) {
    case 2:  reg = 0x60; accelScale = 2.0f  / 4.0f; break;  /* ODR 119, FS 00 */
    case 8:  reg = 0x78; accelScale = 8.0f  / 4.0f; break;  /* ODR 119, FS 11 */
    case 16: reg = 0x68; accelScale = 16.0f / 4.0f; break;  /* ODR 119, FS 01 */
    default:
      Serial.println(F("[IMU] ACCEL_RANGE_G must be 2, 4, 8 or 16 — using 4 g"));
      accelScale = 1.0f;
      return false;
  }
  Wire1.beginTransmission(0x6B);   /* LSM9DS1 accel/gyro, on Wire1 internally */
  Wire1.write(0x20);               /* CTRL_REG6_XL */
  Wire1.write(reg);
  return (Wire1.endTransmission() == 0);
#endif
}

/* Reads the accelerometer if a new sample is ready; otherwise reuses the last
 * one and counts it as stale. Returns the vector norm in g. */
static float readAccel(float *ax, float *ay, float *az) {
  if (IMU.accelerationAvailable()) {
    float x, y, z;
    IMU.readAcceleration(x, y, z);
    lastAx = x * accelScale;
    lastAy = y * accelScale;
    lastAz = z * accelScale;
  } else {
    nStale++;
  }
  *ax = lastAx;
  *ay = lastAy;
  *az = lastAz;

  if (fabsf(lastAx) >= clipThresholdG ||
      fabsf(lastAy) >= clipThresholdG ||
      fabsf(lastAz) >= clipThresholdG) {
    nClipped++;
  }
  return sqrtf(lastAx * lastAx + lastAy * lastAy + lastAz * lastAz);
}

/* Pushes one deviation into the sliding window and returns the window's
 * standard deviation — the AC metric the trigger runs on. O(1) per sample. */
static float acPush(float dev) {
  if (acCount == AC_WINDOW) {
    const float old = acBuf[acHead];
    acSum   -= old;
    acSumSq -= old * old;
  } else {
    acCount++;
  }
  acBuf[acHead] = dev;
  acSum   += dev;
  acSumSq += dev * dev;
  acHead   = (uint16_t)((acHead + 1) % AC_WINDOW);

  /* The incremental add/subtract above drifts as float rounding error piles up
   * over hours. Once per lap of the ring, recompute exactly — that is one pass
   * over AC_WINDOW floats per second, which is nothing. */
  if (acHead == 0) {
    float s = 0.0f, s2 = 0.0f;
    for (uint16_t i = 0; i < acCount; i++) { s += acBuf[i]; s2 += acBuf[i] * acBuf[i]; }
    acSum = s; acSumSq = s2;
  }

  const float inv  = 1.0f / (float)acCount;
  const float mean = acSum * inv;
  float var = acSumSq * inv - mean * mean;
  if (var < 0.0f) var = 0.0f;          /* rounding can push it just below zero */
  return sqrtf(var);
}

/* Seeds the resting vector norm so `dev` sits at ~0 when the machine is still,
 * whatever orientation the board is mounted in. Only a seed — trackBaseline()
 * below owns the value from then on. */
static void calibrateBaseline() {
#if AUTO_CALIBRATE_BASELINE
  Serial.print(F("[CAL] Measuring gravity baseline (keep the board still)... "));

  /* Accumulate centred on a first reading, for the same float32 reason the AC
   * metric is centred: a sum of squares of numbers near 1.0 cannot resolve a
   * variance near 1e-6. */
  float ref = 0.0f, sum = 0.0f, sumSq = 0.0f, lo = 1e9f, hi = -1e9f;
  uint16_t got = 0;
  const uint32_t deadline = millis() + (CAL_SAMPLES * 1000UL / SAMPLE_RATE_HZ) + 2000UL;

  while (got < CAL_SAMPLES && millis() < deadline) {
    if (!IMU.accelerationAvailable()) continue;
    float x, y, z;
    IMU.readAcceleration(x, y, z);
    const float n = sqrtf(x * x + y * y + z * z) * accelScale;
    if (got == 0) ref = n;
    const float d = n - ref;
    sum   += d;
    sumSq += d * d;
    if (n < lo) lo = n;
    if (n > hi) hi = n;
    got++;
  }

  if (got < CAL_SAMPLES / 2) {
    gravityBaseline     = 1.0f;
    baselineProvisional = true;
    Serial.println(F("FAILED — no IMU samples. Using 1.0000 g provisionally."));
    return;
  }

  const float mean = sum / (float)got;
  float var = sumSq / (float)got - mean * mean;
  if (var < 0.0f) var = 0.0f;
  const float sd = sqrtf(var);

  gravityBaseline = ref + mean;
  Serial.print(F("seeded at "));
  Serial.print(gravityBaseline, 4);
  Serial.print(F(" g from "));
  Serial.print(got);
  Serial.print(F(" samples, AC "));
  Serial.print(sd * 1000.0f, 2);
  Serial.println(F(" mg"));

  /* SEED GUARD. Judge the boot window with the same metric the trigger uses,
   * not with peak-to-peak: if the machine is already running, or the board is
   * being handled, this seed is measuring vibration and must not be trusted.
   * It is still USED — a measured value beats an assumed one, and with the
   * trigger now on the DC-free AC metric a wrong baseline no longer breaks
   * detection at all, it only skews the logged `dev` column. It is simply
   * marked provisional, and snapped at the first quiet stretch. */
  if (sd >= VIBE_THRESHOLD) {
    baselineProvisional = true;
    Serial.println(F("      NOT TRUSTED: something was vibrating during the boot"));
    Serial.println(F("      window — machine already running, or the board was"));
    Serial.println(F("      being handled. Marked provisional; it will snap to the"));
    Serial.print(F("      true resting value after "));
    Serial.print(BASELINE_SNAP_QUIET_MS / 1000UL);
    Serial.println(F(" s of quiet."));
  } else {
    baselineProvisional = false;
    if ((hi - lo) >= CAL_MAX_SPREAD_G) {
      Serial.print(F("      note: peak-to-peak was "));
      Serial.print(hi - lo, 4);
      Serial.println(F(" g (a spike, but the window was quiet overall)"));
    }
  }
#else
  gravityBaseline     = 1.0f;
  baselineProvisional = true;
#endif
  /* Reset the stats the calibration reads polluted. */
  nStale = 0;
  nClipped = 0;
}

/* ==========================================================================
 * DS3231 real-time clock (optional — see USE_DS3231)
 *
 * The whole point of the RTC on this rig is not pretty filenames, it is that
 * the device runs unattended on a wall adapter. Without absolute time a power
 * cut is invisible: millis() restarts near zero and "the machine was idle for
 * 40 minutes" becomes indistinguishable from "the board was off for 40
 * minutes". Those mean opposite things and the logs cannot tell them apart.
 * ========================================================================== */

struct RtcTime { uint16_t year; uint8_t mon, day, hour, min, sec; };

#if USE_DS3231

static uint8_t bcd2dec(uint8_t b) { return (uint8_t)((b >> 4) * 10 + (b & 0x0F)); }
static uint8_t dec2bcd(uint8_t d) { return (uint8_t)(((d / 10) << 4) | (d % 10)); }

static bool rtcReadReg(uint8_t reg, uint8_t *dst, uint8_t n) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t)DS3231_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) dst[i] = (uint8_t)Wire.read();
  return true;
}

static bool rtcRead(RtcTime *t) {
  uint8_t r[7];
  if (!rtcReadReg(0x00, r, 7)) return false;
  t->sec  = bcd2dec(r[0] & 0x7F);
  t->min  = bcd2dec(r[1] & 0x7F);
  t->hour = bcd2dec(r[2] & 0x3F);        /* assumes 24 h mode, which we set */
  t->day  = bcd2dec(r[4] & 0x3F);
  t->mon  = bcd2dec(r[5] & 0x1F);
  t->year = (uint16_t)(2000 + bcd2dec(r[6]));
  return (t->mon >= 1 && t->mon <= 12 && t->day >= 1 && t->day <= 31);
}

/* Bit 7 of the status register latches whenever the oscillator has stopped —
 * i.e. the backup cell died or was never fitted. If it is set the stored time
 * is meaningless, however plausible it looks. */
static bool rtcTimeTrustworthy() {
  uint8_t s;
  if (!rtcReadReg(0x0F, &s, 1)) return false;
  return (s & 0x80) == 0;
}

static bool rtcSet(uint16_t Y, uint8_t M, uint8_t D, uint8_t h, uint8_t m, uint8_t s) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.write(dec2bcd(s));
  Wire.write(dec2bcd(m));
  Wire.write(dec2bcd(h));                /* bit 6 clear = 24 hour mode */
  Wire.write((uint8_t)1);                /* day-of-week, unused */
  Wire.write(dec2bcd(D));
  Wire.write(dec2bcd(M));
  Wire.write(dec2bcd((uint8_t)(Y % 100)));
  if (Wire.endTransmission() != 0) return false;

  /* Clear the oscillator-stop flag now that the time is known good. */
  uint8_t st;
  if (rtcReadReg(0x0F, &st, 1)) {
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write((uint8_t)0x0F);
    Wire.write((uint8_t)(st & 0x7F));
    Wire.endTransmission();
  }
  return true;
}
#endif  /* USE_DS3231 */

/* Writes "2025-08-17 19:40:00" into dst (>=20 bytes), or "no-rtc" if there is
 * no trustworthy time. Always safe to call. */
static void rtcStamp(char *dst, size_t cap) {
#if USE_DS3231
  RtcTime t;
  if (rtcOk && rtcRead(&t)) {
    snprintf(dst, cap, "%04u-%02u-%02u %02u:%02u:%02u",
             t.year, t.mon, t.day, t.hour, t.min, t.sec);
    return;
  }
#endif
  snprintf(dst, cap, "no-rtc");
}

/* ==========================================================================
 * SD card / file management
 * ========================================================================== */

static void enterError(const char *why) {
  Serial.print(F("[ERR] "));
  Serial.print(millis());
  Serial.print(F(" ms: "));
  Serial.println(why);
  state = ST_ERROR;
  lastSdRetryMs = millis();
}

/* Strips any leading path the SD library may prepend to a directory entry. */
static const char *baseName(const char *path) {
  const char *slash = strrchr(path, '/');
  return slash ? slash + 1 : path;
}

/* Scans the card root for LOGnnnn.CSV and returns the highest nnnn found
 * (0 if none), so a new session never overwrites an earlier one. */
static uint16_t highestLogIndex() {
  uint16_t highest = 0;

  File root = SD.open("/");
  if (!root) {
    Serial.println(F("[SD ] Could not open root — starting numbering at 1"));
    return 0;
  }

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;

    if (!entry.isDirectory()) {
      const char *nm = baseName(entry.name());
      /* Expect exactly "LOGnnnn.CSV" */
      if (strlen(nm) == 11 &&
          strncasecmp(nm, "LOG", 3) == 0 &&
          strcasecmp(nm + 7, ".CSV") == 0) {
        uint16_t idx = 0;
        bool numeric = true;
        for (uint8_t i = 3; i < 7; i++) {
          if (nm[i] < '0' || nm[i] > '9') { numeric = false; break; }
          idx = idx * 10 + (uint16_t)(nm[i] - '0');
        }
        if (numeric && idx > highest) highest = idx;
      }
    }
    entry.close();
  }
  root.close();
  return highest;
}

/* SD.begin() collapses three completely different failures into one `false`:
 * the SPI/card handshake, mounting the FAT volume, and opening the root dir.
 * Those have opposite fixes (wiring vs. formatting), so on the first failure
 * we re-run the layers individually and say which one actually broke.
 * Only runs on the failure path — it costs nothing when the card works. */
static void reportSdFailure() {
#if !HAVE_SD_LAYER_PROBE
  /* SdFat-based SD library: no Sd2Card/SdVolume to probe with. */
  Serial.println(F("[SD ] init failed. The layer-by-layer probe needs the"));
  Serial.println(F("      classic Arduino SD library and this build has the"));
  Serial.println(F("      SdFat-based one, so run SdRawProbe.ino instead —"));
  Serial.println(F("      it uses no SD library at all and reports the raw"));
  Serial.println(F("      bytes the card puts on MISO."));
#else
  Sd2Card  card;
  SdVolume volume;

  Serial.println(F("[SD ] ---- init failure detail ----"));

  bool cardOk = card.init(SPI_HALF_SPEED, SD_CS_PIN);
  if (!cardOk) {
    Serial.print(F("[SD ] LAYER 1 (SPI / card handshake) FAILED, errorCode=0x"));
    Serial.println(card.errorCode(), HEX);
    Serial.println(F("      retrying at quarter SPI speed..."));
    cardOk = card.init(SPI_QUARTER_SPEED, SD_CS_PIN);
    if (cardOk) {
      Serial.println(F("      ...OK at quarter speed => SPI clock too fast."));
      Serial.println(F("      Shorten the jumper wires (<10 cm) and retry."));
    }
  }

  if (!cardOk) {
    Serial.println(F("      The card never answered at all, so this is NOT a"));
    Serial.println(F("      formatting or partition problem. Suspect, in order:"));
    Serial.println(F("      1. module VCC — a 5 V module fed from 3V3 browns out"));
    Serial.println(F("      2. CS pin mismatch (SD_CS_PIN is set to D10)"));
    Serial.println(F("      3. MISO not returning to 3.3 V, or MOSI/MISO swapped"));
    Serial.println(F("      4. dead card / bad contact"));
    return;
  }

  Serial.print(F("[SD ] LAYER 1 ok — card type: "));
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:  Serial.println(F("SD1"));       break;
    case SD_CARD_TYPE_SD2:  Serial.println(F("SD2"));       break;
    case SD_CARD_TYPE_SDHC: Serial.println(F("SDHC/SDXC")); break;
    default:                Serial.println(F("unknown"));   break;
  }

  const uint32_t megabytes = card.cardSize() / 2048UL;   /* 512 B blocks -> MB */
  Serial.print(F("      capacity ~"));
  Serial.print(megabytes);
  Serial.println(F(" MB"));
  if (megabytes > 32768UL) {
    Serial.println(F("      WARNING: >32 GB means SDXC. The SD library only"));
    Serial.println(F("      supports SD/SDHC up to 32 GB — use a smaller card."));
  }

  if (!volume.init(card)) {
    Serial.println(F("[SD ] LAYER 2 (FAT volume) FAILED"));
    Serial.println(F("      The card responds, so the WIRING IS FINE. This is"));
    Serial.println(F("      the partition table / filesystem. Probing each"));
    Serial.println(F("      MBR partition slot:"));
    for (uint8_t p = 1; p <= 4; p++) {
      SdVolume v;
      Serial.print(F("        partition "));
      Serial.print(p);
      Serial.print(F(": "));
      if (v.init(card, p)) {
        Serial.print(F("FAT"));
        Serial.print(v.fatType());
        Serial.println(p == 1 ? F("") : F("  <-- must be moved to slot 1"));
      } else {
        Serial.println(F("no FAT16/FAT32 volume"));
      }
    }
    Serial.println(F("      The library reads MBR slot 1 only, and cannot read"));
    Serial.println(F("      GPT or exFAT. Card must be a single MBR FAT16/FAT32"));
    Serial.println(F("      partition."));
    return;
  }

  Serial.print(F("[SD ] LAYER 2 ok — FAT"));
  Serial.println(volume.fatType());
  Serial.println(F("[SD ] LAYER 3 (root directory) is the remaining suspect —"));
  Serial.println(F("      the filesystem is probably corrupt; reformat."));
#endif  /* HAVE_SD_LAYER_PROBE */
}

static bool initSd() {
  if (SD.begin(SD_CS_PIN)) return true;

  /* Detail once, not on every 5 s retry. */
  static bool detailPrinted = false;
  if (!detailPrinted) {
    detailPrinted = true;
    reportSdFailure();
  }
  return false;
}

/* Opens a new log file and writes the header row.
 *
 * With a trustworthy RTC the name is MMDDHHMM.CSV — the 8.3 short-name limit
 * the SD library enforces leaves exactly eight characters, so the year does
 * not fit. It is recorded in BOOTLOG.CSV and in the serial log instead. A
 * collision needs two cycles starting in the same minute, which QUIET_TIMEOUT
 * makes impossible; if one somehow happens we fall back to the sequential
 * name rather than overwrite anything. */
static bool openLogFile() {
#if USE_DS3231
  if (rtcOk) {
    RtcTime rt;
    if (rtcRead(&rt)) {
      snprintf(logName, sizeof(logName), "%02u%02u%02u%02u.CSV",
               rt.mon, rt.day, rt.hour, rt.min);
      if (!SD.exists(logName)) {
        logFile = SD.open(logName, FILE_WRITE);
        if (logFile) {
          if (logFile.println(F("millis,ax,ay,az,dev,ac")) == 0) {
            logFile.close();
            enterError("Failed to write CSV header (card full?)");
            return false;
          }
          logFile.flush();
          return true;
        }
      }
      Serial.println(F("[SD ] timestamped name taken — using sequential"));
    }
  }
#endif
  uint16_t idx = highestLogIndex() + 1;

  /* highestLogIndex() only looks at well-formed names, so double-check the
   * chosen name really is free before truncating anything. */
  while (idx <= MAX_LOG_INDEX) {
    snprintf(logName, sizeof(logName), "LOG%04u.CSV", idx);
    if (!SD.exists(logName)) break;
    idx++;
  }
  if (idx > MAX_LOG_INDEX) {
    enterError("SD full of logs (LOG9999.CSV reached)");
    return false;
  }

  logFile = SD.open(logName, FILE_WRITE);
  if (!logFile) {
    enterError("Failed to open new log file");
    return false;
  }

  if (logFile.println(F("millis,ax,ay,az,dev,ac")) == 0) {
    logFile.close();
    enterError("Failed to write CSV header (card full or write-protected?)");
    return false;
  }
  logFile.flush();
  return true;
}

/* Pushes the RAM buffer to the card. `force` also forces a FAT/directory
 * update so the file survives a yank of the USB cable. */
static bool pushBuffer(bool force) {
  if (logBufLen > 0) {
    const size_t written = logFile.write((const uint8_t *)logBuf, logBufLen);
    if (written != logBufLen) {
      logBufLen = 0;
      return false;                 /* card full or gone */
    }
    logBufLen = 0;
    nSinceWrite = 0;
  }
  if (force) {
    logFile.flush();
    lastFlushMs = millis();
  }
  return true;
}

static void startSession() {
  if (!openLogFile()) return;

  logBufLen      = 0;
  nSamples       = 0;
  nSinceWrite    = 0;
  nStale         = 0;
  nDropped       = 0;
  nClipped       = 0;
  sessionPeakAc  = 0.0f;
  sessionPeakDev = 0.0f;
  sessionStartMs = millis();
  lastFlushMs    = sessionStartMs;
  lastActiveMs   = sessionStartMs;
  state          = ST_LOGGING;

  Serial.print(F("[>>>] "));
  Serial.print(sessionStartMs);
  Serial.print(F(" ms: IDLE -> LOGGING, file "));
  Serial.print(logName);
  Serial.print(F(", AC level "));
  Serial.print(acRms * 1000.0f, 2);
  Serial.println(F(" mg"));
}

static void endSession(const char *reason) {
  const bool ok = pushBuffer(true);
  logFile.close();

  const uint32_t now  = millis();
  const uint32_t dur  = now - sessionStartMs;

  Serial.print(F("[<<<] "));
  Serial.print(now);
  Serial.print(F(" ms: LOGGING -> IDLE ("));
  Serial.print(reason);
  Serial.println(F(")"));

  Serial.print(F("      file="));      Serial.print(logName);
  Serial.print(F(" duration="));       Serial.print(dur / 1000UL);
  Serial.print(F("s samples="));       Serial.print(nSamples);
  Serial.print(F(" peakAC="));         Serial.print(sessionPeakAc * 1000.0f, 1);
  Serial.print(F(" mg"));
  Serial.println();
  Serial.print(F("      peakDev="));   Serial.print(sessionPeakDev, 4);
  Serial.print(F(" g baseline="));     Serial.print(gravityBaseline, 4);
  Serial.println(F(" g"));

  Serial.print(F("      dropped="));   Serial.print(nDropped);
  Serial.print(F(" stale="));          Serial.print(nStale);
  Serial.print(F(" clipped="));        Serial.print(nClipped);
  if (dur > 0) {
    Serial.print(F(" effective_rate="));
    Serial.print((float)nSamples * 1000.0f / (float)dur, 1);
    Serial.print(F(" Hz"));
  }
  Serial.println();

  if (nClipped > 0) {
    Serial.print(F("      WARNING: "));
    Serial.print(nClipped);
    Serial.print(F(" samples reached +/-"));
    Serial.print(ACCEL_RANGE_G);
    Serial.println(F(" g full scale — raise ACCEL_RANGE_G to 8"));
  }

  if (!ok) {
    enterError("Final flush failed — data may be truncated");
  } else {
    state = ST_IDLE;
  }
}

/* One line per boot. On a mains-powered rig this is the only record that an
 * outage happened at all: compare each boot time against the last sample in
 * the previous log file and the gap is the blind window. */
static void appendBootLog() {
  File f = SD.open("BOOTLOG.CSV", FILE_WRITE);
  if (!f) {
    Serial.println(F("[SD ] could not open BOOTLOG.CSV"));
    return;
  }
  if (f.size() == 0) f.println(F("datetime,rtc_ok,baseline_g,seed_quiet"));

  char stamp[24];
  rtcStamp(stamp, sizeof(stamp));
  f.print(stamp);
  f.print(',');
#if USE_DS3231
  f.print(rtcOk ? '1' : '0');
#else
  f.print('0');
#endif
  f.print(',');
  f.print(gravityBaseline, 4);
  f.print(',');
  f.println(baselineProvisional ? '0' : '1');
  f.flush();
  f.close();

  Serial.print(F("[SD ] boot recorded in BOOTLOG.CSV at "));
  Serial.println(stamp);
}

/* ==========================================================================
 * Sampling
 * ========================================================================== */

static void takeSample() {
  float ax, ay, az;
  const float norm = readAccel(&ax, &ay, &az);
  const float dev  = norm - gravityBaseline;      /* signed, see header */

  /* The trigger runs on the AC metric: the rolling standard deviation of the
   * norm. It contains no DC term, so an imperfect baseline shifts `dev` but
   * cannot move `acRms` at all — which is the whole point of the change. */
  acRms = acPush(dev);

  const uint32_t now = millis();

  /* How long have we been continuously quiet? Drives both the seed guard and
   * the slow tracker below. */
  if (acCount == AC_WINDOW && acRms < VIBE_THRESHOLD) {
    if (quietSinceMs == 0) quietSinceMs = now;
  } else {
    quietSinceMs = 0;
  }

  /* SEED GUARD. If the boot window was not quiet — power came back mid-cycle,
   * or someone was holding the board — the seed is only provisional. Snap it
   * to the true resting norm at the first properly quiet stretch instead of
   * waiting minutes for the slow tracker to walk there.
   *
   * acSum is the running sum of (norm - gravityBaseline) over the window, so
   * the correction is just its mean. No extra state, no second pass. */
  if (baselineProvisional && quietSinceMs != 0 &&
      (now - quietSinceMs) >= BASELINE_SNAP_QUIET_MS) {
    const float corr = acSum / (float)acCount;
    gravityBaseline += corr;
    if (gravityBaseline < BASELINE_MIN_G) gravityBaseline = BASELINE_MIN_G;
    if (gravityBaseline > BASELINE_MAX_G) gravityBaseline = BASELINE_MAX_G;
    baselineProvisional = false;
    Serial.print(F("[CAL] provisional baseline snapped to "));
    Serial.print(gravityBaseline, 4);
    Serial.print(F(" g after "));
    Serial.print(BASELINE_SNAP_QUIET_MS / 1000UL);
    Serial.print(F(" s quiet (moved "));
    Serial.print(corr * 1000.0f, 1);
    Serial.println(F(" mg)"));
  }

  /* Track the baseline only while the machine is judged still, so a wash cycle
   * can never drag it. Clamped so a transient cannot poison it. */
  if (quietSinceMs != 0) {
    gravityBaseline += dev * baselineAlpha;
    if (gravityBaseline < BASELINE_MIN_G) gravityBaseline = BASELINE_MIN_G;
    if (gravityBaseline > BASELINE_MAX_G) gravityBaseline = BASELINE_MAX_G;
  }

  /* Ignore the first window, before acRms means anything. */
  if (acCount == AC_WINDOW && acRms > VIBE_THRESHOLD) {
    if (aboveSinceMs == 0) aboveSinceMs = now;
    lastActiveMs = now;
  } else {
    aboveSinceMs = 0;
  }

  if (state == ST_LOGGING) {
    if (acRms > sessionPeakAc) sessionPeakAc = acRms;
    if (fabsf(dev) > sessionPeakDev) sessionPeakDev = fabsf(dev);

    /* Flush early if the next line might not fit. */
    if (logBufLen + LINE_MAX > LOG_BUFFER_BYTES) {
      if (!pushBuffer(false)) {
        endSession("SD write failed");
        enterError("SD write failed (card full or removed?)");
        return;
      }
    }

    char *p = logBuf + logBufLen;
    p += appendUInt32(p, now);        *p++ = ',';
    p += appendFixed(p, ax,    4);    *p++ = ',';
    p += appendFixed(p, ay,    4);    *p++ = ',';
    p += appendFixed(p, az,    4);    *p++ = ',';
    p += appendFixed(p, dev,   4);    *p++ = ',';
    p += appendFixed(p, acRms, 5);    *p++ = '\n';
    logBufLen = (size_t)(p - logBuf);

    nSamples++;
    nSinceWrite++;

    const bool byCount = (nSinceWrite >= FLUSH_EVERY_SAMPLES);
    const bool byTime  = ((now - lastFlushMs) >= FLUSH_INTERVAL_MS);
    if (byCount || byTime) {
      if (!pushBuffer(byTime)) {
        endSession("SD write failed");
        enterError("SD write failed (card full or removed?)");
        return;
      }
    }
  }
}

/* ==========================================================================
 * Serial commands
 *
 * The rig normally runs headless on a wall adapter, so these exist for the one
 * bench session where a laptop is attached: set the clock, then walk away.
 *   T2025-08-17 19:40:00   set the RTC
 *   ?                      print current state
 * ========================================================================== */

static void printStatus() {
  char stamp[24];
  rtcStamp(stamp, sizeof(stamp));
  Serial.println(F("---- status ----"));
  Serial.print(F("  time      : ")); Serial.println(stamp);
  Serial.print(F("  state     : "));
  Serial.println(state == ST_IDLE ? F("IDLE") : state == ST_LOGGING ? F("LOGGING") : F("ERROR"));
  Serial.print(F("  AC now    : ")); Serial.print(acRms * 1000.0f, 2);
  Serial.print(F(" mg (threshold ")); Serial.print(VIBE_THRESHOLD * 1000.0f, 1);
  Serial.println(F(" mg)"));
  Serial.print(F("  baseline  : ")); Serial.print(gravityBaseline, 4);
  Serial.println(baselineProvisional ? F(" g  PROVISIONAL") : F(" g  settled"));
  Serial.print(F("  uptime    : ")); Serial.print(millis() / 1000UL); Serial.println(F(" s"));
  if (state == ST_LOGGING) {
    Serial.print(F("  file      : ")); Serial.println(logName);
    Serial.print(F("  samples   : ")); Serial.println(nSamples);
  }
}

static void handleCommand(const char *s) {
  if (s[0] == '?') { printStatus(); return; }

  if (s[0] == 'T' || s[0] == 't') {
#if USE_DS3231
    int Y, M, D, h, m, sec;
    if (sscanf(s + 1, "%d-%d-%d %d:%d:%d", &Y, &M, &D, &h, &m, &sec) == 6 &&
        Y >= 2000 && Y < 2100 && M >= 1 && M <= 12 && D >= 1 && D <= 31 &&
        h < 24 && m < 60 && sec < 60) {
      if (rtcSet((uint16_t)Y, (uint8_t)M, (uint8_t)D, (uint8_t)h, (uint8_t)m, (uint8_t)sec)) {
        rtcOk = rtcTimeTrustworthy();
        char stamp[24];
        rtcStamp(stamp, sizeof(stamp));
        Serial.print(F("[RTC] set to "));
        Serial.println(stamp);
      } else {
        Serial.println(F("[RTC] write failed — check wiring on A4/A5"));
      }
    } else {
      Serial.println(F("[RTC] usage: T2025-08-17 19:40:00"));
    }
#else
    Serial.println(F("[RTC] not built in — set USE_DS3231 to 1 and reflash"));
#endif
    return;
  }
  Serial.println(F("[CMD] unknown. Commands: T<date time>, ?"));
}

/* Non-blocking: a handful of bytes per loop, never waits on the host. */
static void pollSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) { cmdBuf[cmdLen] = '\0'; handleCommand(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

/* ==========================================================================
 * setup() / loop()
 * ========================================================================== */

void setup() {
  ledInit();

  Serial.begin(SERIAL_BAUD);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < SERIAL_WAIT_MS) { /* run headless too */ }

  Serial.println();
  Serial.println(F("=== WasherVibeLogger ==="));
  Serial.print(F("[CFG] rate="));            Serial.print(SAMPLE_RATE_HZ);
  Serial.print(F(" Hz ac_threshold="));      Serial.print(VIBE_THRESHOLD * 1000.0f, 1);
  Serial.print(F(" mg quiet_timeout="));     Serial.print(QUIET_TIMEOUT_MS / 60000UL);
  Serial.print(F(" min range=+/-"));         Serial.print(ACCEL_RANGE_G);
  Serial.print(F(" g cs=D"));                Serial.println(SD_CS_PIN);

  /* Print the SPI mapping the core actually uses instead of trusting a pinout
   * diagram. These macros come from the board variant, so they are correct by
   * construction — wire the SD module to THESE numbers. */
  Serial.print(F("[CFG] SPI per the board variant: MOSI=D"));
  Serial.print(MOSI);
  Serial.print(F(" MISO=D"));
  Serial.print(MISO);
  Serial.print(F(" SCK=D"));
  Serial.print(SCK);
  Serial.print(F(" SS=D"));
  Serial.println(SS);
  Serial.println(F("      Module DI->MOSI, DO->MISO (DI/DO are named from the"));
  Serial.println(F("      card's point of view, so they are easy to swap)."));

  if (!IMU.begin()) {
    Serial.println(F("[IMU] init FAILED"));
    /* Nothing to log without an IMU — sit in ERROR so the LED says so. */
    while (true) {
      state = ST_ERROR;
      ledUpdate();
    }
  }
  Serial.print(F("[IMU] init ok, accel ODR ~"));
  Serial.print(IMU.accelerationSampleRate(), 1);
  Serial.println(F(" Hz"));

  if (!applyAccelRange()) {
    Serial.println(F("[IMU] WARNING: could not set full-scale range"));
  }
  clipThresholdG = (float)ACCEL_RANGE_G * 0.98f;

#if USE_DS3231
  Wire.begin();                       /* external bus, A4/A5 — not the IMU's */
  {
    RtcTime rt;
    if (!rtcRead(&rt)) {
      rtcOk = false;
      Serial.println(F("[RTC] DS3231 not responding on A4/A5 — falling back to"));
      Serial.println(F("      LOGnnnn filenames. Check wiring and pull-ups."));
    } else if (!rtcTimeTrustworthy()) {
      rtcOk = false;
      Serial.println(F("[RTC] oscillator-stop flag set: the stored time is NOT"));
      Serial.println(F("      valid (dead or missing backup cell). Set it with"));
      Serial.println(F("      T2025-08-17 19:40:00 — until then, LOGnnnn names."));
    } else {
      rtcOk = true;
      char stamp[24];
      rtcStamp(stamp, sizeof(stamp));
      Serial.print(F("[RTC] ok, time is "));
      Serial.println(stamp);
    }
  }
#else
  Serial.println(F("[RTC] not built in (USE_DS3231 = 0). Timestamps are"));
  Serial.println(F("      relative to boot, and a power cut will be invisible."));
#endif

  /* Calibrate before touching the SD card: calibration blocks for ~2 s, and if
   * the card is missing we want the red LED lit immediately afterwards rather
   * than a dark board for two seconds. */
  calibrateBaseline();

  if (initSd()) {
    Serial.println(F("[SD ] init ok"));
    appendBootLog();
  } else {
    enterError("SD init FAILED — see detail above");
  }

  nextSampleUs = micros();
  Serial.println(F("[   ] IDLE — waiting for vibration"));
}

void loop() {
  /* --- fixed-rate sampling ------------------------------------------------ */
  const uint32_t now = micros();
  if ((int32_t)(now - nextSampleUs) >= 0) {
    nextSampleUs += sampleIntervalUs;

    /* If an SD write overran the tick budget, skip the missed ticks instead of
     * firing a burst of catch-up samples with bogus timestamps. */
    /* The (missed + 1) matters. Advancing by `missed` intervals leaves
     * nextSampleUs at or just behind micros(), so the very next loop iteration
     * fires again immediately — exactly the bug that put 27,122 samples (1.31%
     * of the first study) 0-1 ms after their predecessor. The extra interval
     * puts the deadline strictly in the future. */
    const int32_t behind = (int32_t)(micros() - nextSampleUs);
    if (behind >= (int32_t)sampleIntervalUs) {
      const uint32_t missed = (uint32_t)behind / sampleIntervalUs;
      nDropped += missed + 1;
      nextSampleUs += (missed + 1) * sampleIntervalUs;
    }

    takeSample();
  }

  /* --- state transitions -------------------------------------------------- */
  const uint32_t nowMs = millis();
  switch (state) {
    case ST_IDLE:
      if (aboveSinceMs != 0 && (nowMs - aboveSinceMs) >= TRIGGER_CONFIRM_MS) {
        startSession();
      }
      break;

    case ST_LOGGING:
      /* Deliberately patient: mid-cycle pauses must NOT end the session. */
      if ((nowMs - lastActiveMs) >= QUIET_TIMEOUT_MS) {
        endSession("quiet timeout");
      }
      break;

    case ST_ERROR:
      if ((nowMs - lastSdRetryMs) >= SD_RETRY_INTERVAL_MS) {
        lastSdRetryMs = nowMs;
        Serial.println(F("[SD ] retrying init..."));
        if (initSd()) {
          Serial.println(F("[SD ] recovered — back to IDLE"));
          aboveSinceMs = 0;      /* don't instantly re-trigger on stale state */
          state = ST_IDLE;
        }
      }
      break;
  }

  pollSerial();
  ledUpdate();
}
