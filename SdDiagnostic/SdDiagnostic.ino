/*
 * SdDiagnostic — isolate why an SD card won't initialise on a Nano 33 BLE
 * ---------------------------------------------------------------------------
 * Run this INSTEAD of WasherVibeLogger when SD init fails. It contains no
 * logger code at all, so whatever it reports is about the card, the wiring and
 * the SD library — not about the logger.
 *
 * It walks the three layers that SD.begin() hides behind a single false:
 *
 *   LAYER 1  SPI + card handshake        -> fails = wiring / power / CS / card
 *   LAYER 2  FAT volume mount            -> fails = partitions / format
 *   LAYER 3  root directory open + list  -> fails = corrupt filesystem
 *
 * Upload, open Serial Monitor at 115200. Send any character to re-run.
 */

#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN 10     /* must match WasherVibeLogger */

/* The SD library exposes the low-level classes used by its own CardInfo
 * example; that is what lets us test one layer at a time. */
Sd2Card  card;
SdVolume volume;
SdFile   root;

static bool tryCard(uint8_t speed, const __FlashStringHelper *label) {
  Serial.print(F("  trying "));
  Serial.print(label);
  Serial.print(F(" ... "));
  if (card.init(speed, SD_CS_PIN)) {
    Serial.println(F("OK"));
    return true;
  }
  Serial.print(F("failed (errorCode 0x"));
  Serial.print(card.errorCode(), HEX);
  Serial.print(F(", data 0x"));
  Serial.print(card.errorData(), HEX);
  Serial.println(F(")"));
  return false;
}

static void runDiagnostic() {
  Serial.println();
  Serial.println(F("================ SD DIAGNOSTIC ================"));
  Serial.print(F("CS pin: D"));
  Serial.println(SD_CS_PIN);
  Serial.println(F("Fixed SPI pins on this board: MOSI=D11 MISO=D12 SCK=D13"));
  Serial.println();

  /* ---------------- LAYER 1: SPI + card handshake ------------------------ */
  Serial.println(F("[LAYER 1] SPI / card handshake"));

  bool ok = tryCard(SPI_HALF_SPEED, F("half speed"));
  bool neededSlowClock = false;
  if (!ok) {
    ok = tryCard(SPI_QUARTER_SPEED, F("quarter speed"));
    neededSlowClock = ok;
  }

  if (!ok) {
    Serial.println();
    Serial.println(F("LAYER 1 FAILED — the card never responded."));
    Serial.println(F("This is NOT a formatting or partition problem; the card"));
    Serial.println(F("is not talking at all. Check, in this order:"));
    Serial.println();
    Serial.println(F(" 1. POWER. This is the usual culprit. A '5V' SD module"));
    Serial.println(F("    with an onboard regulator needs 5V on VCC — from the"));
    Serial.println(F("    Nano's 5V pin, present when USB-powered. Fed from 3V3"));
    Serial.println(F("    its regulator drops below the card's minimum and the"));
    Serial.println(F("    card silently never enumerates. A 3.3V-native module"));
    Serial.println(F("    wants 3V3 instead. Check which one you have."));
    Serial.println(F(" 2. CS. This sketch uses D10; change SD_CS_PIN if needed."));
    Serial.println(F(" 3. MOSI/MISO swapped — very easy to do, and it looks"));
    Serial.println(F("    exactly like this."));
    Serial.println(F(" 4. Jumper length. >10 cm on a breadboard can be enough"));
    Serial.println(F("    to break SPI at these clock rates."));
    Serial.println(F(" 5. GND not shared between module and board."));
    Serial.println(F("==============================================="));
    return;
  }

  if (neededSlowClock) {
    Serial.println();
    Serial.println(F("  NOTE: only worked at the slower clock. That is a signal"));
    Serial.println(F("  integrity problem — shorten the wiring. It may init now"));
    Serial.println(F("  and then corrupt data later at full logging rate."));
  }

  Serial.print(F("  card type: "));
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:  Serial.println(F("SD1"));       break;
    case SD_CARD_TYPE_SD2:  Serial.println(F("SD2"));       break;
    case SD_CARD_TYPE_SDHC: Serial.println(F("SDHC/SDXC")); break;
    default:                Serial.println(F("unknown"));   break;
  }

  const uint32_t megabytes = card.cardSize() / 2048UL;  /* 512 B blocks -> MB */
  Serial.print(F("  capacity: ~"));
  Serial.print(megabytes);
  Serial.println(F(" MB"));
  if (megabytes > 32768UL) {
    Serial.println(F("  *** >32 GB = SDXC. The Arduino SD library supports"));
    Serial.println(F("  *** SD/SDHC up to 32 GB only. Use a 4-32 GB card."));
  }
  Serial.println(F("[LAYER 1] PASSED — wiring and power are good."));
  Serial.println();

  /* ---------------- LAYER 2: FAT volume ---------------------------------- */
  Serial.println(F("[LAYER 2] FAT volume"));

  if (!volume.init(card)) {
    Serial.println(F("  no mountable volume found."));
    Serial.println(F("  probing each MBR partition slot individually:"));
    for (uint8_t p = 1; p <= 4; p++) {
      SdVolume v;
      Serial.print(F("    slot "));
      Serial.print(p);
      Serial.print(F(": "));
      if (v.init(card, p)) {
        Serial.print(F("FAT"));
        Serial.println(v.fatType());
      } else {
        Serial.println(F("empty / not FAT16-32"));
      }
    }
    Serial.println();
    Serial.println(F("LAYER 2 FAILED — wiring is fine, the filesystem is not."));
    Serial.println(F("The library reads MBR partition slot 1 only. It cannot"));
    Serial.println(F("read GPT, and it cannot read exFAT."));
    Serial.println(F("If a slot above shows FAT but slot 1 is empty, your FAT"));
    Serial.println(F("partition is in the wrong slot — repartition as a single"));
    Serial.println(F("primary FAT32 partition (see README)."));
    Serial.println(F("If NO slot shows FAT, the card is exFAT or GPT: reformat."));
    Serial.println(F("==============================================="));
    return;
  }

  Serial.print(F("  FAT type: FAT"));
  Serial.println(volume.fatType());
  Serial.print(F("  cluster size: "));
  Serial.print(volume.blocksPerCluster() * 512UL);
  Serial.println(F(" bytes"));
  Serial.print(F("  volume size: ~"));
  Serial.print((volume.blocksPerCluster() * volume.clusterCount()) / 2048UL);
  Serial.println(F(" MB"));
  Serial.println(F("[LAYER 2] PASSED"));
  Serial.println();

  /* ---------------- LAYER 3: root directory ------------------------------ */
  Serial.println(F("[LAYER 3] root directory"));
  if (!root.openRoot(volume)) {
    Serial.println(F("LAYER 3 FAILED — volume mounts but the root directory"));
    Serial.println(F("will not open. The filesystem is corrupt; reformat."));
    Serial.println(F("==============================================="));
    return;
  }
  Serial.println(F("  files on card:"));
  root.ls(LS_R | LS_SIZE);
  root.close();
  Serial.println(F("[LAYER 3] PASSED"));
  Serial.println();

  /* ---------------- Full stack via the normal API ------------------------ */
  Serial.println(F("[FINAL] SD.begin() as the logger calls it: "));
  Serial.println(SD.begin(SD_CS_PIN) ? F("  SUCCESS — the card is good, put the logger back on.")
                                     : F("  FAILED despite all layers passing (report this)."));
  Serial.println(F("==============================================="));
}

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 5000UL) { }
  runDiagnostic();
  Serial.println(F("Send any character to re-run."));
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    runDiagnostic();
    Serial.println(F("Send any character to re-run."));
  }
}
