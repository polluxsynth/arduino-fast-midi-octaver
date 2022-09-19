/*
 * Fast SSD1306 text printing class.
 * Copyright (c) 2022, Ricard Wanderlof
 * Original code (c) 2012, Adafruit Industries (see fast_ssd1306.ino).
 */

#ifndef FAST_SSD1306_H
#define FAST_SSD1306_H

#define SSD1306_EXTERNALVCC 0x01  ///< External display voltage source
#define SSD1306_SWITCHCAPVCC 0x02 ///< Gen. display voltage from 3.3V

class Fast_SSD1306
{
public:
  Fast_SSD1306(uint8_t w, uint8_t h, TwoWire *twi,
               int8_t rst_pin, uint32_t clk_during = 400000UL,
               uint32_t clk_after = 100000UL);

protected:
  /* Copied from Adafruit_SSD1306 */
  void ssd1306_commandList(const uint8_t *c, uint8_t n);

  /* Copied from Adafruit_SSD1306 */
  void ssd1306_command1(uint8_t c);

public:
  bool begin(uint8_t vcs = SSD1306_SWITCHCAPVCC, uint8_t addr = 0,
             bool reset = true, bool periph_begin = true);

  // Print characters directly to display, starting with
  // coordinates x, y.
  void printDirect(uint8_t x, uint8_t y, const char *s, byte fontsize = 1);

protected:
  uint8_t m_vccstate;      ///< VCC selection, set by begin method.
  uint8_t m_contrast;      ///< normal contrast setting for this device
  uint8_t m_height;        ///< Display height (pixels)
  uint8_t m_width;         ///< Display width (pixels)
  TwoWire *m_wire;         ///< Pointer to two-wire handler
  uint8_t m_i2caddr;       ///< I2C address initialized when begin method is called.
  uint8_t m_rst_pin;       ///< Display reset pin assignment. Set during construction.
  uint32_t m_wire_clk;     ///< Wire speed for SSD1306 transfers
  uint32_t m_restore_clk;  ///< Wire speed following SSD1306 transfers
};

#endif /* FAST_SSD1306_H */
