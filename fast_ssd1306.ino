/*
 * Fast SSD1306 text printing class.
 * Copyright (c) 2022, Ricard Wanderlof
 * Original code (c) 2012, Adafruit Industries (see below)
 */

/* We are obliged to include the following since this is a derivative work: */
/*!
 * @file Adafruit_SSD1306.cpp
 *
 * @mainpage Arduino library for monochrome OLEDs based on SSD1306 drivers.
 *
 * @section intro_sec Introduction
 *
 * This is documentation for Adafruit's SSD1306 library for monochrome
 * OLED displays: http://www.adafruit.com/category/63_98
 *
 * These displays use I2C or SPI to communicate. I2C requires 2 pins
 * (SCL+SDA) and optionally a RESET pin. SPI requires 4 pins (MOSI, SCK,
 * select, data/command) and optionally a reset pin. Hardware SPI or
 * 'bitbang' software SPI are both supported.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a
 * href="https://github.com/adafruit/Adafruit-GFX-Library"> Adafruit_GFX</a>
 * being present on your system. Please make sure you have installed the latest
 * version before using this library.
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries, with
 * contributions from the open source community.
 *
 * @section license License
 *
 * BSD license, all text above, and the splash screen included below,
 * must be included in any redistribution.
 *
 */

/* We use the 6x8 (5x7) LCD type font from glcdfont.c from the Adafruit GFX
 * library. For some reason, we need to include an .h file from the Adafruit
 * GFX Library first, or else we get "file not found" for glcdfont.c.
 * Some Arduino trickery no doubt. */
#include <gfxfont.h>
#include <glcdfont.c>
/* As per the Adafruit license as described in Adafruit_SSD1306.cpp, we are
 * obliged to include the splash screen, however, we are also permitted to
 * opt-out using the following #define. */
/* Since we actually don't have any dependency on Adafruit_SSD1306, but are
 * obliged to include the splash screen in any form of redistribution, we
 * include it here, which means that it actually is a dependency, if only
 * for formal reasons. */
#define SSD1306_NO_SPLASH
#include <splash.h>

/* Although the Adafruit_SSD1306 and associate Adafruit_GFX_Library are comprehensive,
 * they have two (well, three) issues in some contexts:
 * - Printing the whole display to the display memory is too slow when printing MIDI
 *   data on byte by byte basis.
 * - Even when adding routines (displayRect, R.I.P.) to print only a small portion of
 *   the display, the actual printing of fonts is rather cumbersome, printing each
 *   pixel separately using drawPixel().
 * - The screen memory occupies a precious 1kbyte of the available 2kbytes of RAM on the
 *   Arduino UNO.
 *
 * Therefore the Fast_SSD1306 library has been devised. Its function is significantly
 * reduced compared to the Adafruit library: It can only print text, starting at
 * y coordinates that are multiple of 8, in font sizes 1, 2, 3, 4 or 8 . That's it.
 * On the plus side, it's very fast, as each 8 byte vertical row of pixels is written
 * in one go to the display's RAM, and there is no screen memory required. Simple
 * block graphics are of course possible using the existing font.
 *
 * Originally, Fast_SSD1306 was a superclass of Adafruit_SSD1306, however it was felt
 * that this resulted in a bit too much dependencies on the inner workings of that
 * library, so the methods begin(), displayList() and displayCommand1() have been
 * blatently copied, as have a number of #defines, so that the Fast_SSD1306 class
 * can now stand on its own two feet, except for the inclusion of the font definition
 * in glcdfont.h Adafruits contribution is hereby gratefully acknowledged, without
 * which Fast_SSD1306 would likely not have come about.
 *
 * Another limitation is the removal of SPI code since I have no interest in it nor
 * the ability to properly test it. It should be easy to restore from the Adafruit
 * library should the need arise.
 */

// SSD1306 register addresses
#define SSD1306_MEMORYMODE 0x20          ///< See datasheet
#define SSD1306_COLUMNADDR 0x21          ///< See datasheet
#define SSD1306_PAGEADDR 0x22            ///< See datasheet
#define SSD1306_SETCONTRAST 0x81         ///< See datasheet
#define SSD1306_CHARGEPUMP 0x8D          ///< See datasheet
#define SSD1306_SEGREMAP 0xA0            ///< See datasheet
#define SSD1306_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SSD1306_DISPLAYALLON 0xA5        ///< Not currently used
#define SSD1306_NORMALDISPLAY 0xA6       ///< See datasheet
#define SSD1306_INVERTDISPLAY 0xA7       ///< See datasheet
#define SSD1306_SETMULTIPLEX 0xA8        ///< See datasheet
#define SSD1306_DISPLAYOFF 0xAE          ///< See datasheet
#define SSD1306_DISPLAYON 0xAF           ///< See datasheet
#define SSD1306_COMSCANINC 0xC0          ///< Not currently used
#define SSD1306_COMSCANDEC 0xC8          ///< See datasheet
#define SSD1306_SETDISPLAYOFFSET 0xD3    ///< See datasheet
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5  ///< See datasheet
#define SSD1306_SETPRECHARGE 0xD9        ///< See datasheet
#define SSD1306_SETCOMPINS 0xDA          ///< See datasheet
#define SSD1306_SETVCOMDETECT 0xDB       ///< See datasheet

#define SSD1306_SETLOWCOLUMN 0x00  ///< Not currently used
#define SSD1306_SETHIGHCOLUMN 0x10 ///< Not currently used
#define SSD1306_SETSTARTLINE 0x40  ///< See datasheet

#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26              ///< Init rt scroll
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27               ///< Init left scroll
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29 ///< Init diag scroll
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A  ///< Init diag scroll
#define SSD1306_DEACTIVATE_SCROLL 0x2E                    ///< Stop scroll
#define SSD1306_ACTIVATE_SCROLL 0x2F                      ///< Start scroll
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3             ///< Set scroll range

/* Conveniance macros */
#define TRANSACTION_START m_wire->setClock(m_wire_clk);
#define TRANSACTION_END m_wire->setClock(m_restore_clk);
#define WIRE_WRITE m_wire->write
#if defined(I2C_BUFFER_LENGTH)
#define WIRE_MAX min(256, I2C_BUFFER_LENGTH) ///< Particle or similar Wire lib
#elif defined(BUFFER_LENGTH)
#define WIRE_MAX min(256, BUFFER_LENGTH) ///< AVR or similar Wire lib
#elif defined(SERIAL_BUFFER_SIZE)
#define WIRE_MAX                                                               \
  min(255, SERIAL_BUFFER_SIZE - 1) ///< Newer Wire uses RingBuffer
#else
#define WIRE_MAX 32 ///< Use common Arduino core default
#endif

// Don't define this. The macro's usage just indicates code that
// could be used to address a screen memory, if it were used, i.e.
// if this class were integrated into the Adafruit_SSD1306 class.
#undef SCREEN_MEMORY

/* Constructor */
Fast_SSD1306::Fast_SSD1306(uint8_t w, uint8_t h, TwoWire *twi,
                           int8_t rst_pin, uint32_t clk_during,
                           uint32_t clk_after)
  : m_width(w), m_height(h), m_wire(twi ? twi : &Wire),
    m_wire_clk(clk_during), m_restore_clk(clk_after)
  {
  }

/* Copied from Adafruit_SSD1306.cpp */
void Fast_SSD1306::ssd1306_commandList(const uint8_t *c, uint8_t n) {
  if (m_wire) { // I2C
    m_wire->beginTransmission(m_i2caddr);
    WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
    uint16_t bytesOut = 1;
    while (n--) {
      if (bytesOut >= WIRE_MAX) {
        m_wire->endTransmission();
        m_wire->beginTransmission(m_i2caddr);
        WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
        bytesOut = 1;
      }
      WIRE_WRITE(pgm_read_byte(c++));
      bytesOut++;
    }
    m_wire->endTransmission();
  }
}

/* Copied from Adafruit_SSD1306.cpp */
void Fast_SSD1306::ssd1306_command1(uint8_t c) {
  if (m_wire) { // I2C
    m_wire->beginTransmission(m_i2caddr);
    WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
    WIRE_WRITE(c);
    m_wire->endTransmission();
  }
}

/* It would be nice to call the initialization in the Adafruit superclass without buffer
 * setup and splash screen, but unfortunately they are not separate functions, so
 * we just shamelessly copy the actual initialization. */
bool Fast_SSD1306::begin(uint8_t vcs, uint8_t addr,
                         bool reset, bool periph_begin)
{
  m_vccstate = vcs;

  // Setup pin directions
  if (m_wire) { // Using I2C
    // If I2C address is unspecified, use default
    // (0x3C for 32-pixel-tall displays, 0x3D for all others).
    m_i2caddr = addr ? addr : ((m_height == 32) ? 0x3C : 0x3D);
    // TwoWire begin() function1 might be already performed by the calling
    // function if it has unusual circumstances (e.g. TWI variants that
    // can accept different SDA/SCL pins, or if two SSD1306 instances
    // with different addresses -- only a single begin() is needed).
    if (periph_begin)
      m_wire->begin();
  }
  // Reset SSD1306 if requested and reset pin specified in constructor
  // (Technically not needed in our specific case, but let's retain it for future use.
  if (reset && (m_rst_pin >= 0)) {
    pinMode(m_rst_pin, OUTPUT);
    digitalWrite(m_rst_pin, HIGH);
    delay(1);                   // VDD goes high at start, pause for 1 ms
    digitalWrite(m_rst_pin, LOW);  // Bring reset low
    delay(10);                  // Wait 10 ms
    digitalWrite(m_rst_pin, HIGH); // Bring out of reset
  }

  TRANSACTION_START

  // Init sequence
  static const uint8_t PROGMEM init1[] = {SSD1306_DISPLAYOFF,         // 0xAE
                                          SSD1306_SETDISPLAYCLOCKDIV, // 0xD5
                                          0x80, // the suggested ratio 0x80
                                          SSD1306_SETMULTIPLEX}; // 0xA8
  ssd1306_commandList(init1, sizeof(init1));
  ssd1306_command1(m_height - 1);

  static const uint8_t PROGMEM init2[] = {SSD1306_SETDISPLAYOFFSET, // 0xD3
                                          0x0,                      // no offset
                                          SSD1306_SETSTARTLINE | 0x0, // line #0
                                          SSD1306_CHARGEPUMP};        // 0x8D
  ssd1306_commandList(init2, sizeof(init2));

  ssd1306_command1((m_vccstate == SSD1306_EXTERNALVCC) ? 0x10 : 0x14);

  static const uint8_t PROGMEM init3[] = {SSD1306_MEMORYMODE, // 0x20
                                          0x00, // 0x0 act like ks0108
                                          SSD1306_SEGREMAP | 0x1,
                                          SSD1306_COMSCANDEC};
  ssd1306_commandList(init3, sizeof(init3));

  uint8_t comPins = 0x02;
  m_contrast = 0x8F;

  if ((m_width == 128) && (m_height == 32)) {
    comPins = 0x02;
    m_contrast = 0x8F;
  } else if ((m_width == 128) && (m_height == 64)) {
    comPins = 0x12;
    m_contrast = (m_vccstate == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF;
  } else if ((m_width == 96) && (m_height == 16)) {
    comPins = 0x2; // ada x12
    m_contrast = (m_vccstate == SSD1306_EXTERNALVCC) ? 0x10 : 0xAF;
  } else {
    // Other screen varieties -- TBD
  }

  ssd1306_command1(SSD1306_SETCOMPINS);
  ssd1306_command1(comPins);
  ssd1306_command1(SSD1306_SETCONTRAST);
  ssd1306_command1(m_contrast);

  ssd1306_command1(SSD1306_SETPRECHARGE); // 0xd9
  ssd1306_command1((m_vccstate == SSD1306_EXTERNALVCC) ? 0x22 : 0xF1);
  static const uint8_t PROGMEM init5[] = {
      SSD1306_SETVCOMDETECT, // 0xDB
      0x40,
      SSD1306_DISPLAYALLON_RESUME, // 0xA4
      SSD1306_NORMALDISPLAY,       // 0xA6
      SSD1306_DEACTIVATE_SCROLL,
      SSD1306_DISPLAYON}; // Main screen turn on
  ssd1306_commandList(init5, sizeof(init5));

  TRANSACTION_END

  return true;
}

// Print characters directly to display, starting with
// coordinates x, y.
void Fast_SSD1306::printDirect(uint8_t x, uint8_t y, const char *s, byte fontsize)
{
  /* Tables describing how the 8 vertically ordered pixels for each font
   * column are expanded for the different font sizes.
   * For power-of-two font sizes, the expansion of each group of bits (pixels)
   * ends up in separate target bytes, but for the others, some pixels
   * end up in two vertically adjacent target bytes, hence, separate expansion
   * tables are needed for each scanline.
   * For instance, for font size 3, because of the *3 expansion, the first
   * and last scan lines expand the first three and last three bits,
   * respectively, but the middle scan line expands four bits, because it
   * includes pixels from both the first and third scanlines.
   * While all this could likely be done programmatically instead of using
   * tables, tables would be seem to be faster and less error prone than a
   * lot of bit shifting and masking. */
  static const uint8_t PROGMEM expand2[] = { 0x00, 0x03, 0x0c, 0x0f,
                                             0x30, 0x33, 0x3c, 0x3f,
                                             0xc0, 0xc3, 0xcc, 0xcf,
                                             0xf0, 0xf3, 0xfc, 0xff };
  static const uint8_t PROGMEM expand3_0[] = { 0x00, 0x07, 0x38, 0x3f,
                                               0xc0, 0xc7, 0xf8, 0xff };
  static const uint8_t PROGMEM expand3_1[] = { 0x00, 0x01, 0x0e, 0x0f,
                                               0x70, 0x71, 0x7e, 0x7f,
                                               0x80, 0x81, 0x8e, 0x8f,
                                               0xf0, 0xf1, 0xfe, 0xff };
  static const uint8_t PROGMEM expand3_2[] = { 0x00, 0x03, 0x1c, 0x1f,
                                               0xe0, 0xe3, 0xfc, 0xff };
  static const uint8_t PROGMEM expand4[] = { 0x00, 0x0f, 0xf0, 0xff };
#if 0 /* This is actually faster to do programmatically; just leave for illustration */
  static const uint8_t PROGMEM expand8[] = { 0x00, 0xff };
#endif
  const char *s1 = s; /* save pointer to start of string */
  byte x1 = x;
  byte l = strlen(s);
  byte e = x + l * 6 * fontsize;
  uint16_t y1 = y / 8;

  if (e > m_width)
    e = m_width;

  TRANSACTION_START
  // Set up area to write: Only the bytes we are actually going to address
  m_wire->beginTransmission(m_i2caddr);
  WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
  WIRE_WRITE(SSD1306_PAGEADDR);
  WIRE_WRITE(y1);
  WIRE_WRITE(0xff); // not really, but works here
  WIRE_WRITE(SSD1306_COLUMNADDR);
  WIRE_WRITE(x);    // column start address
  WIRE_WRITE(e - 1); // column end address
  m_wire->endTransmission();

  uint16_t count = (e - x) * fontsize;
  uint16_t count1 = 1;
#ifdef SCREEN_MEMORY
  uint8_t *ptr = buffer + m_width * y1 + x;
#endif
  /* A 'scanline' here is not a line of pixels, it corresponds to a line of
   * 8 vertically arranged pixels (as that is the smallest unit that we write
   * to the display).
   * Since the fontsize is the number of screen pixels per font pixel,
   * the number of scanlines is the same as the fontsize.
   * The string to be printed is read several times over, once per scanline,
   * during which the relevant pixels from the font are expanded (if necessary
   * to the screen pixels).
   * Scancolumns ('scancol') similarly correspond to the number of identical
   * sets of 8 vertical pixels that are written in succession in order to expand
   * the font to the fontsize.
   */
  uint16_t scanline = 0, scancol = 0;
  byte data;

  m_wire->beginTransmission(m_i2caddr);
  WIRE_WRITE((uint8_t)0x40);
  uint16_t bytesOut = 1;
  /* PROGMEM */ const byte *fp;
  while (count--) {
    if (bytesOut >= WIRE_MAX) {
      m_wire->endTransmission();
      m_wire->beginTransmission(m_i2caddr);
      WIRE_WRITE((uint8_t)0x40);
      bytesOut = 1;
    }
    if (scancol == 0) { /* generate data for new column, else reuse same data */
      if (--count1 == 0) {
        uint16_t ch = *(byte *)s++; /* fetch character from string */
        fp = &font[ch * 5]; /* font pointer */
        count1 = 6;
      }
      if (count1 == 1)
        data = 0; /* blank last column */
      else {
        data = pgm_read_byte(fp++); /* read font column */
        switch (fontsize) {
          case 1: break;
          case 2: data = pgm_read_byte(&expand2[(data >> (4 * scanline)) & 0xf]);
                  break;
          case 3: switch (scanline) {
                    case 0: data = pgm_read_byte(&expand3_0[data & 0x7]); break;
                    case 1: data = pgm_read_byte(&expand3_1[(data & 0x3c) >> 2]); break;
                    case 2: data = pgm_read_byte(&expand3_2[(data & 0xe0) >> 6]); break;
                  }
                  break;
          case 4: data = pgm_read_byte(&expand4[(data >> (2 * scanline)) & 0x3]);
                  break;
          case 8: data = ((data >> scanline) & 0x1) ? 0xff : 0;
                  break;
          default: break;
        }
      }
    }
#ifdef SCREEN_MEMORY
    *ptr++ = data; /* write to our backup display RAM */
#endif
    WIRE_WRITE(data);
    bytesOut++;
    x++;
    if (++scancol >= fontsize || x >= m_width) {
      scancol = 0;
      if (count1 == 1 && *s == '\0' || x >= m_width) {
        if (++y1 >= m_height / 8)
          break;
        s = s1; /* restart string */
        x = x1; /* restart column index */
        count1 = 1; /* restart font column counter */
        scanline++;
#ifdef SCREEN_MEMORY
        ptr = buffer + m_width * y1 + x; /* display RAM is contiguous; buffer not */
#endif
      }
    }
  }
  m_wire->endTransmission();
  TRANSACTION_END
}

#ifdef SCREEN_MEMORY
// Only write out portion of display, governed by starting
// coordinates x, y and width and hight w, h.
void Fast_SSD1306::displayRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
  TRANSACTION_START
  // Set up area to write: Only the bytes we are actually going to address
  // We assume here that the command length is < WIRE_MAX
  m_wire->beginTransmission(m_i2caddr);
  WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
  WIRE_WRITE(SSD1306_PAGEADDR);
  WIRE_WRITE(y / 8);
  WIRE_WRITE(0xff); // not really, but works here
  WIRE_WRITE(SSD1306_COLUMNADDR);
  WIRE_WRITE(x);    // column start address
  WIRE_WRITE(x + w - 1); // column end address
  m_wire->endTransmission();

  uint16_t count = w * ((h + 7) / 8);
  uint16_t wcount = w;
  uint16_t y1 = y / 8;
  uint8_t *ptr = buffer + m_width * y1 + x;

  m_wire->beginTransmission(i2caddr);
  WIRE_WRITE((uint8_t)0x40);
  uint16_t bytesOut = 1;
  while (count--) {
    if (bytesOut >= WIRE_MAX) {
      m_wire->endTransmission();
      m_wire->beginTransmission(i2caddr);
      WIRE_WRITE((uint8_t)0x40);
      bytesOut = 1;
    }
    WIRE_WRITE(*ptr++);
    bytesOut++;
    if (--wcount == 0) {
      // wrap column, so calculate new buffer address
      y1++;
      ptr = buffer + m_width * y1 + x;
      wcount = w;
    }
  }
  m_wire->endTransmission();
  TRANSACTION_END
}
#endif
