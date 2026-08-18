/*
 * SdRawProbe — talk to the SD card by hand, with no SD library involved
 * ---------------------------------------------------------------------------
 * Run this when SdDiagnostic reports LAYER 1 / errorCode 0x1 (CMD0 timeout)
 * and you have already swapped modules and cards.
 *
 * SdDiagnostic still goes through the SD library, so a library bug and a dead
 * card look identical. This sketch drives SPI directly and PRINTS THE ACTUAL
 * BYTES coming back on MISO, which distinguishes:
 *
 *   all 0xFF  -> line idle high, nothing is answering (power / CS / not seated)
 *   all 0x00  -> line held low   (MISO miswired, shorted, module unpowered)
 *   0x01      -> THE CARD WORKS. The SD library is the problem, not the card.
 *   mixture   -> partial comms, clocking or signal integrity
 *
 * It also prints the core's real pin numbers for MOSI/MISO/SCK/SS, so you can
 * confirm the real mapping rather than trusting any documentation.
 *
 * Upload, open Serial Monitor at 115200. Send any character to re-run.
 */

/* NO SD LIBRARY IS INVOLVED — only SPI. That is the entire point: this sketch
 * is unaffected by which SD library you have installed, or whether it works.
 * Do not add SD.h or SdFat.h here. */
#include <SPI.h>
#include <string.h>

#define SD_CS_PIN 10

/* Slow and forgiving — this is the SD spec's init speed range (100-400 kHz). */
static const SPISettings kInitSpi(250000, MSBFIRST, SPI_MODE0);

/* ------------------------------------------------------------------------ */

static void reportPins() {
  Serial.println(F("[PINS] What the core actually thinks the SPI pins are:"));
  Serial.print(F("  MOSI = D"));  Serial.println(MOSI);
  Serial.print(F("  MISO = D"));  Serial.println(MISO);
  Serial.print(F("  SCK  = D"));  Serial.println(SCK);
  Serial.print(F("  SS   = D"));  Serial.println(SS);
  Serial.print(F("  CS in use = D")); Serial.println(SD_CS_PIN);
  Serial.println(F("  These come from the board variant, so they are correct"));
  Serial.println(F("  by construction. Expected: MOSI=11 MISO=12 SCK=13."));
  Serial.println(F("  If they match, the BOARD side is fine and any wiring"));
  Serial.println(F("  fault is on the MODULE side — check its silkscreen, which"));
  Serial.println(F("  on many breakouts is on the underside only."));
  Serial.println();
}

/* Static read of the MISO line before SPI takes the pin over. Detects a line
 * that is being held low, which SPI traffic alone cannot distinguish from a
 * silent card. */
static void probeMisoIdle() {
  Serial.println(F("[MISO] Static line test (before SPI.begin):"));

  pinMode(MISO, INPUT_PULLUP);
  delay(5);
  const int withPullup = digitalRead(MISO);

  pinMode(MISO, INPUT_PULLDOWN);
  delay(5);
  const int withPulldown = digitalRead(MISO);

  pinMode(MISO, INPUT);
  delay(5);

  Serial.print(F("  with pull-up:   "));   Serial.println(withPullup ? F("HIGH") : F("LOW"));
  Serial.print(F("  with pull-down: "));   Serial.println(withPulldown ? F("HIGH") : F("LOW"));

  if (withPullup == LOW) {
    Serial.println(F("  *** MISO IS BEING HELD LOW. The internal pull-up cannot"));
    Serial.println(F("  *** lift it, so something is actively sinking it:"));
    Serial.println(F("  ***   - module has no power (many hold MISO low unpowered)"));
    Serial.println(F("  ***   - MISO shorted to GND, or wired to the wrong pin"));
    Serial.println(F("  *** Fix this first; nothing else can work until it floats."));
  } else if (withPulldown == HIGH) {
    Serial.println(F("  Line is actively driven HIGH (module is powered and"));
    Serial.println(F("  driving). Normal for a buffered module."));
  } else {
    Serial.println(F("  Line floats (follows the pull) — normal for a card"));
    Serial.println(F("  sitting idle with CS high."));
  }
  Serial.println();
}

/* Sends one SD command and returns the R1 response, tracing every byte. */
static uint8_t sdCommand(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t *trace) {
  SPI.transfer(0xFF);
  SPI.transfer(0x40 | cmd);
  SPI.transfer((uint8_t)(arg >> 24));
  SPI.transfer((uint8_t)(arg >> 16));
  SPI.transfer((uint8_t)(arg >> 8));
  SPI.transfer((uint8_t)arg);
  SPI.transfer(crc);

  uint8_t r = 0xFF;
  for (uint8_t i = 0; i < 10; i++) {
    r = SPI.transfer(0xFF);
    if (trace) trace[i] = r;
    if ((r & 0x80) == 0) break;   /* MSB clear = valid R1 */
  }
  return r;
}

static void runProbe() {
  Serial.println();
  Serial.println(F("============== RAW SD SPI PROBE =============="));

  reportPins();
  probeMisoIdle();

  /* ---- SPI init sequence, exactly as the SD spec describes it ---------- */
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  SPI.begin();
  SPI.beginTransaction(kInitSpi);

  /* >=74 clocks with CS high and MOSI high, to wake the card into SPI mode */
  Serial.println(F("[CMD0] sending 80 wake-up clocks with CS high..."));
  for (uint8_t i = 0; i < 10; i++) SPI.transfer(0xFF);

  digitalWrite(SD_CS_PIN, LOW);

  uint8_t trace[10];
  memset(trace, 0xAA, sizeof(trace));    /* 0xAA = "never written" marker */
  const uint8_t r1 = sdCommand(0, 0, 0x95, trace);   /* CMD0, CRC is fixed */

  Serial.print(F("[CMD0] response bytes:"));
  for (uint8_t i = 0; i < 10; i++) {
    Serial.print(F(" 0x"));
    if (trace[i] < 0x10) Serial.print('0');
    Serial.print(trace[i], HEX);
  }
  Serial.println();

  /* ---- Interpret -------------------------------------------------------- */
  bool allFF = true, allZero = true;
  for (uint8_t i = 0; i < 10; i++) {
    if (trace[i] != 0xFF && trace[i] != 0xAA) allFF = false;
    if (trace[i] != 0x00 && trace[i] != 0xAA) allZero = false;
  }

  Serial.println();
  if (r1 == 0x01) {
    Serial.println(F("*** CMD0 SUCCEEDED (R1 = 0x01, card is in idle state)."));
    Serial.println(F("*** THE CARD AND WIRING ARE FINE."));
    Serial.println(F("*** Go run WasherVibeLogger. Only if the SD library still"));
    Serial.println(F("*** fails from here is the library itself a suspect."));

    /* Go one step further: CMD8 tells us the card's voltage support. */
    uint8_t t8[10];
    memset(t8, 0xAA, sizeof(t8));
    const uint8_t r8 = sdCommand(8, 0x1AA, 0x87, t8);
    Serial.print(F("    CMD8 R1 = 0x"));
    Serial.println(r8, HEX);
    if (r8 == 0x01) {
      Serial.print(F("    CMD8 trailing bytes:"));
      for (uint8_t i = 0; i < 4; i++) {
        Serial.print(F(" 0x"));
        Serial.print(SPI.transfer(0xFF), HEX);
      }
      Serial.println(F("   (expect ... 0x1 0xAA = SDv2, 3.3 V OK)"));
    }
  } else if (allZero) {
    Serial.println(F("*** MISO STUCK LOW during the transfer."));
    Serial.println(F("*** The card is not driving the bus. Check, in order:"));
    Serial.println(F("***  1. VCC AT THE MODULE, measured with a meter while"));
    Serial.println(F("***     powered. Continuity does not prove voltage."));
    Serial.println(F("***  2. MISO on the wrong pin — use the MISO number"));
    Serial.println(F("***     printed above, not one from a pinout diagram."));
    Serial.println(F("***  3. MOSI and MISO swapped."));
  } else if (allFF) {
    Serial.println(F("*** NOTHING IS ANSWERING (line idle high the whole time)."));
    Serial.println(F("*** Clocks are going out but no card is replying:"));
    Serial.println(F("***  1. MODULE POWER — see the note below, this is the"));
    Serial.println(F("***     most common cause and it survives swapping"));
    Serial.println(F("***     modules if both are the same type."));
    Serial.println(F("***  2. CS not reaching the module's CS pin (D10 here)."));
    Serial.println(F("***  3. SCK or MOSI not reaching the module."));
    Serial.println(F("***  4. Card not seated in the socket."));
  } else {
    Serial.print(F("*** PARTIAL COMMUNICATION. Last R1 = 0x"));
    Serial.println(r1, HEX);
    Serial.println(F("*** Something is on the bus but not framing correctly —"));
    Serial.println(F("*** suspect signal integrity: shorten the wires."));
  }

  digitalWrite(SD_CS_PIN, HIGH);
  SPI.transfer(0xFF);
  SPI.endTransaction();

  /* Only worth reading when the card did NOT answer. Printing a wall of
   * power-supply advice after a successful probe made a pass look like a
   * failure, which is the opposite of this sketch's job. */
  if (r1 != 0x01) {
    Serial.println();
    Serial.println(F("MEASURE THIS BEFORE ANYTHING ELSE:"));
    Serial.println(F("  VCC at the module's own pins, with a meter, powered up."));
    Serial.println(F("  A '5V' module (AMS1117 regulator + 74LVC125 buffer, the"));
    Serial.println(F("  common blue/HW-125 type) fed from 3V3 outputs only ~2.2 V"));
    Serial.println(F("  to the card because of the regulator's dropout, and the"));
    Serial.println(F("  card never enumerates. That failure is IDENTICAL across"));
    Serial.println(F("  every module and card of that type. Feed VCC from the"));
    Serial.println(F("  Nano's 5V pin instead (live when USB-powered)."));
    Serial.println(F("  Then confirm MISO idles at ~3.3 V, NOT 5 V, before"));
    Serial.println(F("  trusting it — this board is not 5 V tolerant."));
  }
  Serial.println(F("=============================================="));
}

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 5000UL) { }
  runProbe();
  Serial.println(F("Send any character to re-run."));
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    runProbe();
    Serial.println(F("Send any character to re-run."));
  }
}
