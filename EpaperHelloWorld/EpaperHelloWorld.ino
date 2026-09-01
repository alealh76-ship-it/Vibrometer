/*
 * EpaperHelloWorld — three-colour "Hello World" on a WeAct 1.54" BWRY e-paper
 * ---------------------------------------------------------------------------
 * Board : nRF52840 SuperMini (sold since ~2023 as "ProMicro nRF52840"; a
 *         nice!nano-compatible clone with the Adafruit UF2 bootloader)
 *
 *   Arduino IDE setup
 *     File -> Preferences -> Additional Boards Manager URLs:
 *       https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
 *     Tools -> Board -> Boards Manager -> install "Adafruit nRF52 Boards"
 *     Tools -> Board -> Adafruit nRF52 Boards -> "Nordic nRF52840 DK"
 *
 *   Why the DK entry: that package ships no SuperMini/nice!nano variant, and
 *   the DK variant (pca10056) is the honest stand-in because it numbers
 *   Arduino pins exactly like the chip's own GPIOs — Arduino pin n is P0.n,
 *   and P1.n is 32 + n. The P-numbers silkscreened on the SuperMini therefore
 *   translate straight across; see the P0()/P1() macros below. That entry also
 *   uploads with nrfutil serial DFU after a 1200 bps touch, which is exactly
 *   what this board's bootloader speaks, so plain Upload works. If no port
 *   shows up, double-tap RST to drop into the bootloader and pick the new port.
 *   (The DK's own LED/button pins do NOT exist on the SuperMini, so LED_BUILTIN
 *   is meaningless here. This sketch reports status over USB serial instead.)
 *
 * Libs  : GxEPD2 >= 1.6.6   (Library Manager — 1.6.6 is where this panel landed)
 *         Adafruit GFX Library (installed as a GxEPD2 dependency)
 *
 * Display: WeAct 1.54" four-colour e-paper, black / white / red / yellow,
 *          200x200 px. Driven here as Good Display GDEM0154F51H (JD79660
 *          controller), GxEPD2 class GxEPD2_154c_GDEM0154F51H.
 *
 *   CHECK THE RESOLUTION BEFORE BLAMING THE WIRING. There are two 1.54"
 *   four-colour panels in circulation: the 200x200 GDEM0154F51H used here, and
 *   a 152x152 GDEY0154F51 (JD79661) that GxEPD2 1.6.9 does NOT support. A
 *   152x152 panel still compiles against this sketch and still gets clocked
 *   data — it just comes back blank or scrambled.
 *
 * Wiring — the module's 8-pin FPC-side header, in its printed order:
 *
 *     module pin | SuperMini pad | Arduino pin no. | note
 *     -----------+---------------+-----------------+------------------------
 *     1  BUSY    | P0.17         | 17              | panel drives this
 *     2  RES     | P0.20         | 20              | reset, active low
 *     3  D/C     | P0.22         | 22              | data / command select
 *     4  CS      | P0.24         | 24              | chip select, active low
 *     5  SCL     | P1.00         | 32              | SPI clock  (SCK)
 *     6  SDA     | P0.11         | 11              | SPI data in (MOSI)
 *     7  GND     | GND           | —               |
 *     8  VCC     | 3V3           | —               | 3.3 V only, see below
 *
 *   SDA/SCL on this module are SPI, not I2C — the names come from the panel
 *   datasheet. The display never talks back on a data line, so MISO goes
 *   nowhere; the nRF52 SPI peripheral still needs a pin assigned to it, so
 *   P1.04 is parked there and left unconnected. Do not wire anything to it.
 *
 *   Both the panel and the nRF52840 are 3.3 V parts, so this is a direct
 *   connection with no level shifting. Power the module from the SuperMini's
 *   3V3 pad, never from 5V/RAW: the WeAct board has no regulator.
 *
 *   Pins P0.09 and P0.10 are deliberately unused — they default to NFC antenna
 *   function on the nRF52840 and need a chip-level config change to become
 *   plain GPIO.
 *
 * Timing: a four-colour full refresh takes about 25 seconds, during which the
 *   panel flashes through its colour passes. That is normal for BWRY e-paper,
 *   not a hang. The sketch draws once in setup() and then does nothing — Good
 *   Display asks for at least ~180 s between full refreshes on colour panels,
 *   so an update loop is the wrong shape for a hello-world.
 */

#include <GxEPD2_4C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <SPI.h>

// ---------------------------------------------------------------------------
// Pin map
//
// With the pca10056 variant, an Arduino pin number IS the nRF52840 GPIO number:
// port 0 pin n == n, port 1 pin n == 32 + n. These two macros keep the code
// speaking in the P-numbers printed on the board, so what you read here is
// what you read on the silkscreen. Select a different board variant and these
// numbers stop meaning what they say — recheck them against that variant.
// ---------------------------------------------------------------------------
#define P0(n) (n)
#define P1(n) (32 + (n))

#define EPD_BUSY  P0(17)
#define EPD_RST   P0(20)
#define EPD_DC    P0(22)
#define EPD_CS    P0(24)
#define EPD_SCK   P1(0)
#define EPD_MOSI  P0(11)
#define EPD_MISO  P1(4)   // unused by the panel; SPI needs a pin, leave it open

// One page holding the whole screen: 200 x 200 px at 2 bits per pixel is
// 10 kB, nothing to the nRF52840's 256 kB of RAM, so the drawing loop below
// runs exactly one pass.
GxEPD2_4C<GxEPD2_154c_GDEM0154F51H, GxEPD2_154c_GDEM0154F51H::HEIGHT> display(
  GxEPD2_154c_GDEM0154F51H(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

static const char HELLO[] = "Hello World";

// Baselines for the three lines, in pixels from the top of the 200 px panel.
static const int16_t LINE_BLACK_Y  = 78;
static const int16_t LINE_RED_Y    = 118;
static const int16_t LINE_YELLOW_Y = 158;

// Draw text horizontally centred, with `y` as the baseline.
static void drawCentred(const char* text, int16_t y, uint16_t colour)
{
  int16_t  x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  display.setCursor((display.width() - int16_t(w)) / 2 - x1, y);
  display.setTextColor(colour);
  display.print(text);
}

// Same, but on a filled bar. Yellow ink on white paper is genuinely faint on
// these panels, so the yellow line gets a black bar behind it — otherwise the
// third colour reads as "nothing printed" from any distance.
static void drawCentredOnBar(const char* text, int16_t y, uint16_t colour, uint16_t barColour)
{
  int16_t  x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  const int16_t x = (display.width() - int16_t(w)) / 2 - x1;
  display.fillRoundRect(x + x1 - 8, y1 - 6, int16_t(w) + 16, int16_t(h) + 12, 6, barColour);
  display.setCursor(x, y);
  display.setTextColor(colour);
  display.print(text);
}

static void drawHelloWorld()
{
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);

    // A red double frame, so the border proves the red channel independently
    // of the text.
    display.drawRect(0, 0, display.width(), display.height(), GxEPD_RED);
    display.drawRect(1, 1, display.width() - 2, display.height() - 2, GxEPD_RED);

    // Caption in the built-in 5x7 font.
    display.setFont(NULL);
    display.setTextSize(1);
    display.setTextColor(GxEPD_BLACK);
    {
      const char* caption = "WeAct 1.54\" BWRY";
      int16_t  x1, y1;
      uint16_t w, h;
      display.getTextBounds(caption, 0, 0, &x1, &y1, &w, &h);
      display.setCursor((display.width() - int16_t(w)) / 2, 26);
      display.print(caption);
    }

    // The same words in each of the three inks the panel can lay down.
    display.setFont(&FreeMonoBold9pt7b);
    drawCentred(HELLO, LINE_BLACK_Y, GxEPD_BLACK);
    drawCentred(HELLO, LINE_RED_Y, GxEPD_RED);
    drawCentredOnBar(HELLO, LINE_YELLOW_Y, GxEPD_YELLOW, GxEPD_BLACK);
  }
  while (display.nextPage());
}

void setup()
{
  Serial.begin(115200);
  // Serial is USB CDC on this board. Give a host a moment to enumerate, but
  // never block: the board has to run the same when nothing is plugged in.
  const unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 3000)) {}
  Serial.println(F("EpaperHelloWorld: start"));

  // Bind the SPI peripheral to the pads we actually wired. This has to happen
  // before display.init(), which is what calls SPI.begin().
  SPI.setPins(EPD_MISO, EPD_SCK, EPD_MOSI);

  // 115200 -> GxEPD2 prints its own diagnostics to Serial.
  // true    -> full init (this is a cold boot, not a wake from deep sleep).
  // 20      -> reset pulse in ms, the library default.
  // false   -> drive RST normally rather than the pull-down trick some
  //            5 V Waveshare boards need.
  display.init(115200, true, 20, false);
  display.setRotation(0);
  display.setTextWrap(false);

  Serial.println(F("refreshing; a four-colour full update takes ~25 s"));
  const unsigned long refreshStart = millis();
  drawHelloWorld();
  Serial.print(F("refresh done in "));
  Serial.print(millis() - refreshStart);
  Serial.println(F(" ms"));

  // Park the panel in deep sleep. The image stays on screen with no power.
  display.hibernate();
  Serial.println(F("panel hibernating; image is retained"));
}

void loop()
{
  // Nothing to do. E-paper holds the last image without power, and this panel
  // does not want another full refresh for a few minutes anyway.
}
