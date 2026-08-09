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
 *   millis,ax,ay,az,magnitude
 *   - millis    : millis() at sample time, relative to boot (no RTC)
 *   - ax/ay/az  : g, 4 decimals (LSM9DS1 @ +/-4 g is ~0.000122 g/LSB)
 *   - magnitude : |sqrt(ax^2+ay^2+az^2) - gravityBaseline|, in g
 *                 This is the same signal the trigger uses (before smoothing),
 *                 and it is fully recomputable offline from ax/ay/az.
 *
 * Wiring (SPI pins are fixed on the Nano 33 BLE; only CS is your choice)
 *   SD MOSI -> D11    SD MISO -> D12    SD SCK -> D13    SD CS -> D10
 *   SD VCC  -> see README: the Nano 33 BLE is a 3.3 V board and is NOT 5 V
 *              tolerant. Use a 3.3 V-native microSD breakout, or a 5 V module
 *              whose level shifter also shifts MISO back down to 3.3 V.
 *   NOTE: D13 is SCK here, so LED_BUILTIN is unusable while the SD card is
 *         wired. That is why all status indication uses the onboard RGB LED.
 */

#include <Arduino_LSM9DS1.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <strings.h>   /* strcasecmp / strncasecmp */

/* Built against the Arduino SD library, whose Sd2Card/SdVolume classes the
 * failure diagnostics use. SdFat v2 renamed those (SdCard/FsVolume) and also
 * redefines F(), so it is not a drop-in here — porting to SdFat means changing
 * reportSdFailure() as well as the include. See the README. */
#if !defined(SD_CARD_TYPE_SD1)
#error "Install the Arduino SD library (Library Manager -> 'SD' by Arduino). This sketch does not build against SdFat v2 as-is."
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

/* Vibration threshold, in g, applied to the SMOOTHED magnitude.
 * Start high-ish and lower it after looking at a real idle-vs-wash recording.
 * 0.02 g is a reasonable first guess for a machine on a solid floor. */
#define VIBE_THRESHOLD          0.02f

/* How long the smoothed magnitude must stay above threshold before we open a
 * file. Rejects door bumps, someone leaning on the machine, etc. */
#define TRIGGER_CONFIRM_MS      500UL

/* How long the machine must stay quiet before we close the file and go IDLE.
 * Washers pause for minutes mid-cycle (soak, drain, redistribute), and this
 * study is specifically about characterising those pauses — err long. */
#define QUIET_TIMEOUT_MS        (5UL * 60UL * 1000UL)   /* 5 minutes */

/* Smoothing time constant for the trigger signal, in samples. Larger = calmer
 * trigger, slower response. ~25 samples @100 Hz is a 0.25 s time constant. */
#define VIBE_SMOOTH_SAMPLES     25.0f

/* Accelerometer full-scale range, in g. The stock library is hard-wired to
 * +/-4 g; any other value here is applied by writing CTRL_REG6_XL directly and
 * rescaling the library's readings (see applyAccelRange()). Valid: 2, 4, 8, 16.
 * Leave at 4 unless the clip counter reports clipping during spin. */
#define ACCEL_RANGE_G           4

/* Log buffer. Samples are formatted into this buffer as they are taken and
 * pushed to the SD card in one block, instead of writing per sample. */
#define LOG_BUFFER_BYTES        4096
#define FLUSH_EVERY_SAMPLES     100     /* push buffer to file every N samples */
#define FLUSH_INTERVAL_MS       1000UL  /* ...and force a FAT flush this often */

/* Boot-time gravity baseline calibration. The board's resting orientation sets
 * the DC level of the vector norm; measuring it beats assuming exactly 1.000 g.
 * Rejected (falls back to 1.0 g) if the board is moving during calibration. */
#define AUTO_CALIBRATE_BASELINE 1
#define CAL_SAMPLES             200     /* 2 s @ 100 Hz */
#define CAL_MAX_SPREAD_G        0.05f   /* reject calibration if noisier */

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
static float    gravityBaseline = 1.0f;   /* g, set by calibration */
static float    vibeLevel       = 0.0f;   /* smoothed |norm - baseline|, g */
static uint32_t aboveSinceMs    = 0;      /* 0 = currently below threshold */
static uint32_t lastActiveMs    = 0;      /* last time we were above threshold */

/* Accel scale correction, applied when ACCEL_RANGE_G != 4 (library assumes 4) */
static float    accelScale      = 1.0f;
static float    clipThresholdG  = (float)ACCEL_RANGE_G * 0.98f;

/* Write buffer */
static char     logBuf[LOG_BUFFER_BYTES];
static size_t   logBufLen      = 0;
#define LINE_MAX 72     /* worst-case formatted line length, incl. newline */

/* Per-session statistics, printed on close */
static uint32_t sessionStartMs = 0;
static uint32_t nSamples       = 0;
static uint32_t nSinceWrite    = 0;
static uint32_t lastFlushMs    = 0;
static uint32_t nStale         = 0;   /* IMU had no new data at tick time */
static uint32_t nDropped       = 0;   /* ticks skipped (SD write overran) */
static uint32_t nClipped       = 0;   /* samples at/over full scale */
static float    sessionPeakG   = 0.0f;

static uint32_t lastSdRetryMs  = 0;

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

/* Measures the resting vector norm so the trigger metric sits at ~0 when the
 * machine is still, whatever orientation the board is mounted in. */
static void calibrateBaseline() {
#if AUTO_CALIBRATE_BASELINE
  Serial.print(F("[CAL] Measuring gravity baseline (keep the board still)... "));

  float sum = 0.0f, lo = 1e9f, hi = -1e9f;
  uint16_t got = 0;
  const uint32_t deadline = millis() + (CAL_SAMPLES * 1000UL / SAMPLE_RATE_HZ) + 2000UL;

  while (got < CAL_SAMPLES && millis() < deadline) {
    if (!IMU.accelerationAvailable()) continue;
    float x, y, z;
    IMU.readAcceleration(x, y, z);
    const float n = sqrtf(x * x + y * y + z * z) * accelScale;
    sum += n;
    if (n < lo) lo = n;
    if (n > hi) hi = n;
    got++;
  }

  if (got >= CAL_SAMPLES / 2 && (hi - lo) < CAL_MAX_SPREAD_G) {
    gravityBaseline = sum / (float)got;
    Serial.print(F("ok, baseline = "));
    Serial.print(gravityBaseline, 4);
    Serial.println(F(" g"));
  } else {
    gravityBaseline = 1.0f;
    Serial.print(F("too noisy (spread "));
    Serial.print(hi - lo, 4);
    Serial.println(F(" g) — falling back to 1.0000 g"));
  }
#else
  gravityBaseline = 1.0f;
#endif
  /* Reset the stats the calibration reads polluted. */
  nStale = 0;
  nClipped = 0;
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

/* Opens the next free LOGnnnn.CSV and writes the header row. */
static bool openLogFile() {
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

  if (logFile.println(F("millis,ax,ay,az,magnitude")) == 0) {
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
  sessionPeakG   = 0.0f;
  sessionStartMs = millis();
  lastFlushMs    = sessionStartMs;
  lastActiveMs   = sessionStartMs;
  state          = ST_LOGGING;

  Serial.print(F("[>>>] "));
  Serial.print(sessionStartMs);
  Serial.print(F(" ms: IDLE -> LOGGING, file "));
  Serial.print(logName);
  Serial.print(F(", trigger level "));
  Serial.print(vibeLevel, 4);
  Serial.println(F(" g"));
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
  Serial.print(F(" peak="));           Serial.print(sessionPeakG, 4);
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

/* ==========================================================================
 * Sampling
 * ========================================================================== */

static void takeSample() {
  float ax, ay, az;
  const float norm = readAccel(&ax, &ay, &az);
  const float mag  = fabsf(norm - gravityBaseline);

  /* Exponential moving average — the trigger runs on this, not on `mag`, so a
   * single noisy sample can't start or extend a session. */
  vibeLevel += (mag - vibeLevel) / VIBE_SMOOTH_SAMPLES;

  const uint32_t now = millis();

  if (vibeLevel > VIBE_THRESHOLD) {
    if (aboveSinceMs == 0) aboveSinceMs = now;
    lastActiveMs = now;
  } else {
    aboveSinceMs = 0;
  }

  if (state == ST_LOGGING) {
    if (mag > sessionPeakG) sessionPeakG = mag;

    /* Flush early if the next line might not fit. */
    if (logBufLen + LINE_MAX > LOG_BUFFER_BYTES) {
      if (!pushBuffer(false)) {
        endSession("SD write failed");
        enterError("SD write failed (card full or removed?)");
        return;
      }
    }

    char *p = logBuf + logBufLen;
    p += appendUInt32(p, now);      *p++ = ',';
    p += appendFixed(p, ax,  4);    *p++ = ',';
    p += appendFixed(p, ay,  4);    *p++ = ',';
    p += appendFixed(p, az,  4);    *p++ = ',';
    p += appendFixed(p, mag, 4);    *p++ = '\n';
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
  Serial.print(F(" Hz threshold="));         Serial.print(VIBE_THRESHOLD, 4);
  Serial.print(F(" g quiet_timeout="));      Serial.print(QUIET_TIMEOUT_MS / 1000UL);
  Serial.print(F(" s range=+/-"));           Serial.print(ACCEL_RANGE_G);
  Serial.print(F(" g cs=D"));                Serial.println(SD_CS_PIN);

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

  /* Calibrate before touching the SD card: calibration blocks for ~2 s, and if
   * the card is missing we want the red LED lit immediately afterwards rather
   * than a dark board for two seconds. */
  calibrateBaseline();

  if (initSd()) {
    Serial.println(F("[SD ] init ok"));
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
    const int32_t behind = (int32_t)(micros() - nextSampleUs);
    if (behind >= (int32_t)sampleIntervalUs) {
      const uint32_t missed = (uint32_t)behind / sampleIntervalUs;
      nDropped += missed;
      nextSampleUs += missed * sampleIntervalUs;
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

  ledUpdate();
}
