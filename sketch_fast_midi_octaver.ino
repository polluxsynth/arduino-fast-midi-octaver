#include <string.h>
/* For OLED graphics: */
#include <Adafruit_SSD1306.h>
#include "glcdfont.c"

/*
 * Fast (low latency - 320 us) octave transpose filter, with sustain pedal emulation and MIDI
 * channelize options.
 */

/* Released under GPLv2. See LICENSE for details. */

/*
 * Octave transpose feature:
 * - Octave transpose governed by grounding one of the TRANSPOSE_PIN_n pins, either with a
 *   rotary switch or pushbuttons. Originally, the range is -3, -2, -1, 0, +1 or +2 octaves,
 *   but the lowest range is normally replaced by a settings mode.
 * - In low latency mode, can only handle keyboards where note off is sent with status 128.
 * - When 144 with velocity 0 is received, switches to normal latency mode, where complete note on/off
 *   message must be received before sending it on (latency 960 us).
 *   - The first 144 note off received in low latency mode will not be shifted by the transpose memory.
 *     This is unlikely to be a problem in practice, as is the first note received.
 * - When status 128 is received, switches to low latency mode.
 * - Status LED (D13) flashes 10 ms on incoming data. The status LED is inverted in normal mode
 *   (i.e. normally on, and goes dark 10 ms on incoming data).
 * - In low latency mode, does not remember original channel when note off received,
 *   unless HANDLE_CHANNEL is set. This adds a further 320 us.
 * - Mimics running status of source.
 *
 * Skip CC function. This is intended to skip unwanted CC messages. It was originally devised
 * to avoid passing through CC 22, 23 and 24 representing the accelerometer in the Radikal
 * Technologies Accelerator synth.
 * - When SKIP_CC is set, skips all CC 22, 23 and 24 messages. This feature adds
 *   320 us of latency to all passing CC messages.
 *
 * Sustain Pedal Emulation (SPE). The purpose of this feature is for synths and other devices
 * which have issues processing the sustain pedal. The Waldorf Blofeld is a case in point, as
 * it tends to arbitrarily disregard sustain pedal messages.
 * The limited memory within the Arduino results in some compromises with this mode. Most
 * importantly, each key on the source kan only represent a single note on the destination,
 * which means that whenever a note on from the source is received which is already held,
 * on a different channel or tranpose setting than the original note, the original note is
 * turned off, as there is only memory for one channel and transpose setting per source note.
 *
 * SPE has a number of sub modes:
 * - By default, whenever a note on for the same channel and tranpose setting is received for
 *   a note that is already held, it is just sent on with no further action. When the sustain
 *   pedal is released, a single note off is sent, which tecnically is an inbalance in the
 *   number of note ons and offs sent, but empirically most synths handle this without any
 *   voices hanging - the note off counts for all the previous note ons received for that
 *   note. However, many synths still allocate a new voice for every note on received, even
 *   when the note number is the same as a voice already playing, which a) causes a build-up
 *   of voices when the same note is repated with the sustain pedal held, and b) can cause
 *   the synth to run out of voices. In order to alleviate, when "SPE Max 1" is set, under
 *   the above circumstances, a note off is sent prior to sending the note on, so that
 *   the number of note ons and offs is balanced, and no voice buildup or running out of
 *   voices will occur. This is governed by the MODE_SPE_AVOID_STACKUP mode flag, and
 *   the SEND_NOTE_OFF_FOR_EACH_SUSTAINED_NOTE macro.
 * - By default, if the sustain pedal is down, when notes are received on a new channel or
 *   transposition setting, old notes are sustained as far as possible (the exception being
 *   if the new note has the same source note as one already held, as described previously,
 *   which causes a note off to be sent for the already playing note, as only one destination
 *   note is allowed to exist for each source note, due to memory limitations). However, when
 *   "SPE R All" is set, whenever a note is received and the channel and/or transposition has
 *   changed since the previous note, all previous notes are released. This is governed by the
 *   MODE_SPE_RELEASE_ALL_WHEN_CHANNEL_CHANGED mode flag and the
 *   RELEASE_ALL_NOTES_WHEN_CHANNEL_OR_OCTAVE_CHANGED macro.
 * - By default, whenever the source channel is changed, the sustain pedal state is retained,
 *   i.e. new notes are also held. However, when "SPE P U Ch" is set, whenever a note on is
 *   received on a different channel than the previous message, the sustain pedal is set
 *   to the up (released) state, however, existing held notes are not released. In order to
 *   release them, the pedal must be pressed and released again. This means that notes can be
 *   held on one channel, and switching to a new channel will still hold them, while the notes
 *   on the new channel are played without hold This is governed by the
 *   MODE_SPE_PEDAL_UP_WHEN_CHANNEL_CHANGED flag and DONT_SUSTAIN_WHEN_NOTE_CHANNEL_CHANGED macro.
 * - Sostenuto. By default, all notes are held once the sustain pedal is pressed. In sostenuto
 *   mode, i.e. when "SPE Sostnu" is set, only notes which are currently playing are held.
 *   Notes received after the pedal has been pressed will not sustain. (This mode is not
 *   yet implemented).
 * - Sustain pedal no running status mode. This mode cancels running status when outputting
 *   sustain pedal CC messages, when "PedNoRunSt" is set. This works around a bug in the
 *   Waldorf Blofeld, which causes a pedal down message (CC 64 127) to be disregarded if
 *   running status is in effect, for instance when receiving the sequence CC 64 0 64 127
 *   (pedal up then down), causing notes to unexpectedly not sustain even though the
 *   sustain pedal is depressed.
 * - Channelize. Whenever the "Channelize" parameter is set to "off", the channel of incoming
 *   messages is not changed. When this parameter is set to MIDI channel 1..16, incoming
 *   messages are assigned this channel instead.
 *
 * Parameter setting. When enabled, the settings mode replaces the lowest transpose (-3) mde.
 * In this mode, no MIDI messages are passed through (and the current state of all notes and
 * the sustain pedal (when applicable) is retained), and instead the keys are used to set the
 * different parameters.
 *
 * - Starting at note 36 (i.e. the lowest octave on a 5 octave keyboard), the white notes
 *   select which parameter to set, starting with Channelize on C, SPE on D, and the SPE sub
 *   modes on the following keys.
 * - The corresponding black notes represent parameter values, with C# being off or 0,
 *   D# being on or 1, etc, for two octaves, thus representing the values 0 to 9. In order
 *   to enter MIDI channel numbers above 9, press two keys in succession, e.g. D# (for 1)
 *   followed by F# (for 2) to enter channel number 12.
 * - Parameter selection is retained as long as the devices is switched on. The default
 *   parameter is Channelize after power on.
 * - Parameters are currently note saved in non volatile memory, and default to "off" at
 *   next power on.
 */

/* Define in order to remember note on channel when setting note offs in low latency mode.
 * (channel is always remembered in normal latency mode as it doesn't add any latency).
 * This adds a further 320 us latency to note off messages.
 */
#define HANDLE_CHANNEL

/* Skip CC 22, 23, 24. The Radikal Technologies Accelerator seems to output this spuriously
 * with values of 64 or 65, especially 22, even when the transmit accelerometer parameter
 * is off.
 * Enabling this features adds 320 us of latency to (all) CC messages.
 */
//#define SKIP_CC "\026\027\030" /* CC 22, 23, 24 */

/* Convert sustain pedal to delayed note offs - Sustain Pedal Emulation mode. */
/* All the flags here can be disabled runtime in the settings screens (mode_flags) */

/* EMULATE_SUSTAIN_PEDAL also needs to be set in order to include the optional
 * CC No Running Status (CC_PED_NRS) mode, which is intended to fix a problem with the
 * Waldorf Blofeld which doesn't handle running status for the sustain pedal properly,
 * meaning that if sustain pedal down (CC 64 127) message is received with running
 * status employed, it does not register it, and subsequent notes will not sustain.
 * CC_PED_NRS mode always inserts a CC before every outgoing sustain pedal message, regardless
 * of whether running status is employed by the source or not, or MIRROR_INCOMING_RUNNING_STATUS
 * is set or not.
 */
#define EMULATE_SUSTAIN_PEDAL
/* Sub modes: */
/* When repeated notes received while sustaing for the same note with the same transpose/channel
 * as previous note, send note off to avoid stacking up voices. */
#define SEND_NOTE_OFF_FOR_EACH_SUSTAINED_NOTE
/* Set this to release all sustained notes whenever a note is received with a channel and/or
 * transpose setting differing from the previous note received. */
#define RELEASE_ALL_NOTES_WHEN_CHANNEL_OR_OCTAVE_CHANGED
/* Setting this means that the sustain pedal is considered up once a note on a different channel
 * is received than before. The pedal is not considered to have transitioned from down to up though,
 * so previously held notes are still held, but will be released when the pedal is released. */
#define DONT_SUSTAIN_WHEN_NOTE_CHANNEL_CHANGED

/* Dependencies */

#if defined(SKIP_CC) || defined(EMULATE_SUSTAIN_PEDAL)
#define PROCESS_CC
#endif

#if defined(SEND_NOTE_OFF_FOR_EACH_SUSTAINED_NOTE) || defined(EMULATE_SUSTAIN_PEDAL)
#define HANDLE_CHANNEL
#endif

#undef MIRROR_INCOMING_RUNNING_STATUS
/* Every second, reset running status to avoid long periods without status on output */
#ifndef MIRROR_INCOMING_RUNNING_STATUS
#define RUNNING_STATUS_TIMEOUT_US 1000000
#endif

/* Define this to enable the setting of parameters. When not set, the lowest octave
 * setting will be replaced by the parameter setting mode.
 */
#define ENABLE_PARAMETER_SETTING

/* Define this to enable MIDI dumps. */
#define MIDIDUMP
#define DUMPBUF_SIZE 12

/* Define this to enable runtime parameter display, on an SSD1306 128x64 OLED
 * Note that the setting of parameters is still possible even if there is no display
 * (you just can't see what has actually been set)
 */
#define UI_DISPLAY

#define OCTAVE_OFFSET 4 /* e.g. octave -3 => encoded octave value is 1 */

/* Digital pins 0 and 1 are used for (MIDI) serial communication, so use digital I/O
 * 2 and upwards for the transpose control inputs. The pins are set to pull up,
 * and control is excercised by grounding them, either using a rotary switch
 * or push buttons.
 */
#define TRANSPOSE_PIN_0        2 /* Doubles as mode set when activated */
#define TRANSPOSE_PIN_1        3
#define TRANSPOSE_PIN_2        4
#define TRANSPOSE_PIN_3        5
#define TRANSPOSE_PIN_4        6
#define TRANSPOSE_PIN_5        7

/* LED flash time */
#define LED_FLASH_US 10000

#define STATE_PASS             0
#define STATE_NOTE_ON_NOTE     1
#define STATE_NOTE_ON_VEL      2
#define STATE_NOTE_OFF_NOTE    3
#define STATE_NOTE_OFF_VEL     4
#define STATE_CC_ADDR          5
#define STATE_CC_VAL           6

/* MIDI status bytes */
#define NOTE_OFF 128
#define NOTE_ON 144
#define CONTROL_CHANGE 176
#define CC_SUSTAIN_PEDAL 64
#define REALTIME 248 /* This and above */

#ifdef UI_DISPLAY

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define HEADER_X 0
#define HEADER_Y 0
#define HEADER_TEXTSIZE 2
#define HEADER2_OFFSET (12 * 7)
#define BODY_X 2
#define BODY_Y 24
#define BODY_TEXTSIZE 4
#define DUMP_X (HEADER2_OFFSET - 6)
#define DUMP_Y 16
#define DUMP_TEXTSIZE 1

long int now;

class Fast_SSD1306: public Adafruit_SSD1306
{
public:
  Fast_SSD1306(uint8_t w, uint8_t h, TwoWire *twi,
               int8_t rst_pin, uint32_t clkDuring = 400000UL,
               uint32_t clkAfter = 100000UL)
     : Adafruit_SSD1306(w, h, twi, rst_pin, clkDuring, clkAfter)
     {
     }

#define TRANSACTION_START wire->setClock(wireClk);
#define TRANSACTION_END wire->setClock(restoreClk);
#define WIRE_WRITE wire->write
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

  // Only write out portion of display, governed by starting
  // coordinates x, y and width and hight w, h.
  void displayRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
  {
    TRANSACTION_START
    // Set up area to write: Only the bytes we are actually going to address
    // We assume here that the command length is < WIRE_MAX
    wire->beginTransmission(i2caddr);
    WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
    WIRE_WRITE(SSD1306_PAGEADDR);
    WIRE_WRITE(y / 8);
    WIRE_WRITE(0xff); // not really, but works here
    WIRE_WRITE(SSD1306_COLUMNADDR);
    WIRE_WRITE(x);    // column start address
    WIRE_WRITE(x + w - 1); // column end address
    wire->endTransmission();

    uint16_t count = w * ((h + 7) / 8);
    uint16_t wcount = w;
    uint16_t y1 = y / 8;
    uint8_t *ptr = buffer + WIDTH * y1 + x;

    wire->beginTransmission(i2caddr);
    WIRE_WRITE((uint8_t)0x40);
    uint16_t bytesOut = 1;
    while (count--) {
      if (bytesOut >= WIRE_MAX) {
        wire->endTransmission();
        wire->beginTransmission(i2caddr);
        WIRE_WRITE((uint8_t)0x40);
        bytesOut = 1;
      }
      WIRE_WRITE(*ptr++);
      bytesOut++;
      if (--wcount == 0) {
        // wrap column, so calculate new buffer address
        y1++;
        ptr = buffer + WIDTH * y1 + x;
        wcount = w;
      }
    }
    wire->endTransmission();
    TRANSACTION_END
  }

  // Print characters directly to display, starting with
  // coordinates x, y.
  void printDirect(uint8_t x, uint8_t y, const char *s, byte fontsize = 1)
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

    if (e > WIDTH)
      e = WIDTH;

    TRANSACTION_START
    // Set up area to write: Only the bytes we are actually going to address
    wire->beginTransmission(i2caddr);
    WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
    WIRE_WRITE(SSD1306_PAGEADDR);
    WIRE_WRITE(y1);
    WIRE_WRITE(0xff); // not really, but works here
    WIRE_WRITE(SSD1306_COLUMNADDR);
    WIRE_WRITE(x);    // column start address
    WIRE_WRITE(e - 1); // column end address
    wire->endTransmission();

    uint16_t count = (e - x) * fontsize;
    uint16_t count1 = 1;
    uint8_t *ptr = buffer + WIDTH * y1 + x;
    uint16_t scanline = 0, scancol = 0;
    byte data;

    wire->beginTransmission(i2caddr);
    WIRE_WRITE((uint8_t)0x40);
    uint16_t bytesOut = 1;
    /* PROGMEM */ const byte *fp;
    while (count--) {
      if (bytesOut >= WIRE_MAX) {
        wire->endTransmission();
        wire->beginTransmission(i2caddr);
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
      *ptr++ = data; /* write to our backup display RAM */
      WIRE_WRITE(data);
      bytesOut++;
      x++;
      if (++scancol >= fontsize || x >= WIDTH) {
        scancol = 0;
        if (count1 == 1 && *s == '\0' || x >= WIDTH) {
          if (++y1 >= HEIGHT / 8)
            break;
          s = s1; /* restart string */
          x = x1; /* restart column index */
          count1 = 1; /* restart font column counter */
          scanline++;
          ptr = buffer + WIDTH * y1 + x; /* display RAM is contiguous; buffer not */
        }
      }
    }
    wire->endTransmission();
    TRANSACTION_END
  }

};

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SSD1306_I2C_ADDRESS 0x3c
Fast_SSD1306 disp(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1, 1000000UL);

#endif

byte transpose = 0;
byte octave_encoded = OCTAVE_OFFSET;
byte octave_prev = 0;
bool low_latency_mode = true;

byte channelize = 0;

enum mode_flags {
  MODE_SPE = 0,
  MODE_SPE_AVOID_STACKUP = 1,
  MODE_SPE_RELEASE_ALL_WHEN_CHANNEL_CHANGED = 2,
  MODE_SPE_PEDAL_UP_WHEN_CHANNEL_CHANGED = 3,
  MODE_SPE_SOSTENUTO = 4,
  MODE_CC_PED_NRS = 5,
  MODE_LAST
};

#define MODE_BIT(flag) (1 << (flag))

#define MODE_SET(flag) (mode_flags & MODE_BIT(flag))

byte mode_flags = MODE_BIT(MODE_SPE_AVOID_STACKUP) | MODE_BIT(MODE_CC_PED_NRS); /* default value */

bool setting_parameters = false;
bool setting_parameters_prev = false;

byte settings_screen = 0; /* first screen in settings_screens (Channelize) */

#ifdef MIDIDUMP
/* The idea behind DUMP_TIMOUT_US is to wait a while after receiving MIDI data when
 * hopefully the additional latency resulting from writing to the display will
 * be less disruptive. However, this can go awry if the MIDI input is continually
 * active, e.g. if the source sends us a MIDI clock. Since display printing is now
 * relatively fast, leave the timeout at 0 for now. */
//#define DUMP_TIMEOUT_US 50000 /* 50 ms */
#define DUMP_TIMEOUT_US 0 /* now! */
byte midi_dumpbuf[DUMPBUF_SIZE];
byte dumpbuf_wrpos = -1; /* first write will be att +1, i.e. 0 */
byte dumpbuf_rdpos = 0;
byte dumpbuf_fill = 0;
enum dump_mode { DUMPMODE_OFF = 0, DUMPMODE_IN, DUMPMODE_OUT };
byte dump_mode = DUMPMODE_OFF;
long dump_time_us = 0;

void dump(byte data)
{
  if (++dumpbuf_wrpos >= DUMPBUF_SIZE)
    dumpbuf_wrpos = 0;
  midi_dumpbuf[dumpbuf_wrpos] = data;
  dumpbuf_fill++;
}

#endif

long int led_flash_time;
long int running_status_time;

#define DESC_CHANNEL_MASK 0x0f
#define DESC_OCTAVE_MASK 0x70 /* encoded octave */
#define DESC_OCTAVE_SHIFT 4
#define DESC_SUSTAIN 0x80 /* sustained by sustain pedal */

byte note_descriptors[128] = { 0 };

struct note {
  byte status;
  byte note;
  byte vel;
};

class notebits {

  public:
    notebits(bool initial): m_bits{(byte)(initial ? 255 : 0)}
    {

    }

    notebits(void)
    {
      notebits(0);
    }

    void set(byte bitno)
    {
       m_bits[byteno(bitno)] |= bytebit(bitno);
    }

    void clear(byte bitno)
    {
       m_bits[byteno(bitno)] &= ~bytebit(bitno);
    }

    bool is_set(byte bitno)
    {
      return !!(m_bits[byteno(bitno)] & bytebit(bitno));
    }

    void clearall(void)
    {
      memset(m_bits, 0, 16);
    }

  private:
    byte bytebit(byte bitno)
    {
      return 1 << (bitno & 7);
    }

    byte byteno(byte bitno)
    {
      return (bitno & 127) / 8;
    }

    byte m_bits[128 / 8];
};

notebits sostenuto_held;

byte read_octave_switch(void)
{
  byte new_octave = octave_encoded;

  /* High-transpose priority */
  if (digitalRead(TRANSPOSE_PIN_5) == LOW)
    new_octave = 2 + OCTAVE_OFFSET;
  else if (digitalRead(TRANSPOSE_PIN_4) == LOW)
    new_octave = 1 + OCTAVE_OFFSET;
  else if (digitalRead(TRANSPOSE_PIN_3) == LOW)
    new_octave = 0 + OCTAVE_OFFSET;
  else if (digitalRead(TRANSPOSE_PIN_2) == LOW)
    new_octave = -1 + OCTAVE_OFFSET;
  else if (digitalRead(TRANSPOSE_PIN_1) == LOW)
    new_octave = -2 + OCTAVE_OFFSET;
  else if (digitalRead(TRANSPOSE_PIN_0) == LOW)
    new_octave = -3 + OCTAVE_OFFSET;

  return new_octave;
}

struct screen_def {
#ifdef UI_DISPLAY
  const char *header;
  void (*str_func)(const struct screen_def *, char *);
#endif
  byte *value;
  byte v_shift;
  byte v_mask;
  void (*val_func)(const struct screen_def *, byte);
};

void chan_val(/* PROGMEM */ const struct screen_def *screen, byte keyval)
{
  byte *value_ptr = pgm_read_ptr(&screen->value);
  byte newval = *value_ptr * 10 + keyval;

  if (newval > 16)
    *value_ptr = keyval;
  else
    *value_ptr = newval;
}

void dump_val(/* PROGMEM */ const struct screen_def *screen, byte keyval)
{
  byte *value_ptr = pgm_read_ptr(&screen->value);
  if (keyval < 3)
    *value_ptr = keyval;
}

void bool_val(/* PROGMEM */ const struct screen_def *screen, byte keyval)
{
  byte *value_ptr = pgm_read_ptr(&screen->value);
  byte shift = pgm_read_byte(&screen->v_shift);

  if (keyval) /* anything but 0 */
    *value_ptr |= 1 << shift; /* set it */
  else
    *value_ptr &= ~(1 << shift); /* clear it */
}

#ifdef UI_DISPLAY

byte get_value(/* PROGMEM */ const struct screen_def *screen)
{
  return ((*(byte *)pgm_read_ptr(&screen->value)) >>
           pgm_read_byte(&screen->v_shift)) &
          pgm_read_byte(&screen->v_mask);
}

void octave_str(/* PROGMEM */ const struct screen_def *screen, char *buffer)
{
  byte octave = get_value(screen);
  byte disp_octave;

  if (octave < OCTAVE_OFFSET) {
    *buffer++ = '-';
    disp_octave = OCTAVE_OFFSET - octave; /* always positive */
  } else if (octave > OCTAVE_OFFSET) {
    *buffer++ = '+';
    disp_octave = octave - OCTAVE_OFFSET;
  } else {
    *buffer++ = ' ';
    disp_octave = 0;
  }
  *buffer++ = disp_octave + '0';
  *buffer++ = ' '; /* erase any old character from setup menu */
  *buffer = '\0';
}

const char off_s[] PROGMEM = "off";

void chan_str(/* PROGMEM */ const struct screen_def *screen, char *buffer)
{
  byte value = get_value(screen);
  byte pval = value;

  if (value) {
    if (value >= 10) {
      *buffer++ = '1';
      pval -= 10;
    }
    *buffer++ = pval + '0';
    if (value < 10)
      *buffer++ = ' '; /* erase any old character */
    *buffer++ = ' '; /* erase any old character */
    *buffer = '\0';
  } else {
    strcpy_P(buffer, off_s);
  }
}

void string_str(/* PROGMEM */ const struct screen_def *screen, const char * const text[], char *buffer)
{
  byte value = get_value(screen);

  strcpy_P(buffer, pgm_read_ptr(&text[value]));
}

void bool_str(/* PROGMEM */ const struct screen_def *screen, char *buffer)
{
  static const char on_s[] PROGMEM = "on ";
  static const char *const bool_text[2] PROGMEM = { off_s, on_s };

  string_str(screen, bool_text, buffer);
}

void dump_str(/* PROGMEM */ const struct screen_def *screen, char *buffer)
{
  static const char in_s[] PROGMEM = "in ";
  static const char out_s[] PROGMEM = "out";
  static const char *const dump_text[3] PROGMEM = { off_s, in_s, out_s };

  string_str(screen, dump_text, buffer);
}

/* Put spaces as applicable at end of strings so they erase whatever was on
 * the display before when printed. */
const char octave_s[] PROGMEM = "Octave ";
const char channelize_s[] PROGMEM = "Channelize";
#ifdef EMULATE_SUSTAIN_PEDAL
const char sust_ped_e_s[] PROGMEM = "Sust Ped E";
const char spe_avoid_stackup_s[] PROGMEM = "SPE Max 1 ";
const char spe_r_all_s[] PROGMEM = "SPE R All ";
const char spe_r_ch_s[] PROGMEM = "SPE P U Ch";
const char spe_sost_s[] PROGMEM = "SPE Sostnu";
const char cc_nrs_s[] PROGMEM = "PedNoRunSt";
#endif
#ifdef MIDIDUMP
const char mididump_s[] PROGMEM = "MIDI dump ";
#endif

#endif

PROGMEM const struct screen_def octave_screen =
{
#ifdef UI_DISPLAY
  octave_s, octave_str,
#endif
                        &octave_encoded, 0, 255, NULL };

/* The setup screens are in the order given in settings_screens[] */
PROGMEM const struct screen_def settings_screens[] = {
  {
#ifdef UI_DISPLAY
    channelize_s, chan_str,
#endif
                             &channelize, 0, 255, chan_val },
#ifdef EMULATE_SUSTAIN_PEDAL
  {
#ifdef UI_DISPLAY
    sust_ped_e_s, bool_str,
#endif
                     &mode_flags, MODE_SPE, 1, bool_val },
  {
#ifdef UI_DISPLAY
    spe_avoid_stackup_s, bool_str,
#endif
                                   &mode_flags, MODE_SPE_AVOID_STACKUP, 1, bool_val },
  {
#ifdef UI_DISPLAY
    spe_r_all_s, bool_str,
#endif
                           &mode_flags, MODE_SPE_RELEASE_ALL_WHEN_CHANNEL_CHANGED, 1, bool_val },
  {
#ifdef UI_DISPLAY
    spe_r_ch_s, bool_str,
#endif
                          &mode_flags, MODE_SPE_PEDAL_UP_WHEN_CHANNEL_CHANGED, 1, bool_val },
  {
#ifdef UI_DISPLAY
    spe_sost_s, bool_str,
#endif
                          &mode_flags, MODE_SPE_SOSTENUTO, 1, bool_val },
  {
#ifdef UI_DISPLAY
    cc_nrs_s, bool_str,
#endif
                          &mode_flags, MODE_CC_PED_NRS, 1, bool_val },
#endif
#ifdef MIDIDUMP
  {
#ifdef UI_DISPLAY
    mididump_s, dump_str,
#endif
                          &dump_mode, 0, 3, dump_val },
#endif
};

#ifdef UI_DISPLAY

/* Triple space for clearing triple character items,
 * and also for clearing display, by using fontsize = 8 */
const char triple_space_s[] PROGMEM = "   ";
const char spe_s[] PROGMEM = "SPE";
const char sos_s[] PROGMEM = "Sos";
const char in_s[] PROGMEM = " M\x1b"; /* M<- */
const char out_s[] PROGMEM = " M\x1a"; /* M-> */

static char strbuf[11]; /* used for sundry strings to be printed: header, value, midi dump ... */

#ifdef MIDIDUMP
void display_dump(bool dump_all)
{
  static byte print_pos = 0;

  if (dump_all) {
    /* clear screen area and reset print position */
    disp.printDirect(DUMP_X, DUMP_Y, strcpy_P(strbuf, triple_space_s + 1), 8);
    print_pos = 0;
  }

  while (dumpbuf_fill) {
    byte p = print_pos;

    sprintf(strbuf, "\xf9%-3d", midi_dumpbuf[dumpbuf_rdpos++]);
    disp.printDirect(DUMP_X + (p / 6) * 6 * 4, DUMP_Y + (p % 6) * 8, strbuf, DUMP_TEXTSIZE);
    // Blank out the little dot before the previous current line
    p = print_pos == 0 ? 11 : (print_pos - 1);
    strbuf[0] = ' ';
    strbuf[1] = '\0';
    disp.printDirect(DUMP_X + (p / 6) * 6 * 4, DUMP_Y + (p % 6) * 8, strbuf, DUMP_TEXTSIZE);

    /* Administration */
    if (dumpbuf_rdpos >= DUMPBUF_SIZE)
      dumpbuf_rdpos = 0;
    print_pos++;
    if (print_pos >= DUMPBUF_SIZE)
      print_pos = 0;
    dumpbuf_fill--;

    if (!dump_all) /* Only run once unless printing whole buffer */
      break;
  }
}
#endif

void display_screen(/* PROGMEM */ const struct screen_def *screen)
{
  void (*str_func)(const struct screen_def *, char *);
  static const struct screen_def *prev_screen = NULL;

  disp.printDirect(HEADER_X, HEADER_Y,
                  strcpy_P(strbuf, pgm_read_ptr(&screen->header)), HEADER_TEXTSIZE);

  /* Clear dump area when going from octave -> setup */
  if (screen != &octave_screen && prev_screen == &octave_screen)
    disp.printDirect(DUMP_X, DUMP_Y, strcpy_P(strbuf, triple_space_s + 1), 8);

  /* Display additional information on octave screen: SPE mode, sostenuto, and channelize
   * (when enabled). */
  if (screen == &octave_screen) {
#ifdef MIDIDUMP
    if (dump_mode) {
      /* Print header */
      disp.printDirect(HEADER_X + HEADER2_OFFSET, HEADER_Y,
                       strcpy_P(strbuf, dump_mode == DUMPMODE_IN ? in_s : out_s), HEADER_TEXTSIZE);
    } else
#endif
           {
    /* normal mode: print SPE, Sos and channelize when applicable */
      disp.printDirect(HEADER_X + HEADER2_OFFSET, HEADER_Y,
                       strcpy_P(strbuf, MODE_SET(MODE_SPE) ? spe_s : triple_space_s),
                       HEADER_TEXTSIZE);
      disp.printDirect(HEADER_X + HEADER2_OFFSET, HEADER_Y + 16,
                       strcpy_P(strbuf,
                                MODE_SET(MODE_SPE) && MODE_SET(MODE_SPE_SOSTENUTO) ?
                                sos_s : triple_space_s),
                       HEADER_TEXTSIZE);
      if (channelize) {
        char *s = strbuf;
        *s++ = 'C';
        if (channelize < 10)
          *s++ = 'h';
        itoa(channelize, s, 10);
      } else
        strcpy_P(strbuf, triple_space_s);
      disp.printDirect(HEADER_X + HEADER2_OFFSET, HEADER_Y + 32 + 8, strbuf, HEADER_TEXTSIZE);

    }
  }
  str_func = pgm_read_ptr(&screen->str_func);
  str_func(screen, strbuf);
  disp.printDirect(BODY_X, BODY_Y, strbuf, BODY_TEXTSIZE);
  prev_screen = screen;
}
#endif

byte apply_note_on_transpose(byte note, byte channel)
{
  byte note_transpose = transpose;

  /* Overrange management - shift back till we get within valid range */
  while ((note + note_transpose) & 128) {
    /* Transpose is byte type, so unsigned, so we'll handle the sign ourselves */
    if (note_transpose & 128) /* negative */
      note_transpose += 12; /* too low, so try an octave higher */
    else
      note_transpose -= 12; /* too high, so try an octave lower */
  }

  /* This also clears the DESC_SUSTAIN bit. */
  note_descriptors[note] = (octave_encoded << DESC_OCTAVE_SHIFT) | channel;

  return note + note_transpose;
}

byte apply_note_off_transpose(byte note, bool clear_descriptor_entry)
{
  byte ret = note;

  if (note_descriptors[note]) /* only perform transpose if descriptor has been set */
    ret += ((note_descriptors[note] & DESC_OCTAVE_MASK) >> DESC_OCTAVE_SHIFT) * 12 - OCTAVE_OFFSET * 12;

  if (clear_descriptor_entry) { /* Mark the note as released */
    note_descriptors[note] = 0;
    sostenuto_held.clear(note);
  }

  return ret;
}

void serial_write(byte data)
{
  Serial.write(data);
#ifdef MIDIDUMP
  if (dump_mode == DUMPMODE_OUT) {
    dump(data);
    dump_time_us = now;
  }
#endif
}

class RunningStatus {

public:
  /* Constructor */
  RunningStatus(void) : m_running_status(0), m_last_status(0)
  {
  }

  /* Send status byte, honoring running status, unless fresh_status_byte is set.
   * Return true if byte was actually sent, false otherwise
   */
  bool send(byte status, bool fresh_status_byte)
  {
    if (status != m_running_status || fresh_status_byte) {
      m_running_status = m_last_status = status;
      serial_write(status);
      return true;
    }
    return false;
  }

  /* When fresh_status_byte not set it defaults to false */
  bool send(byte status)
  {
    return send(status, false);
  }

  /* Return last sent sent status */
  byte last_status(void)
  {
    return m_last_status;
  }

  void clear(void)
  {
    m_running_status = 0;
  }

private:
  byte m_running_status;
  byte m_last_status;
};

RunningStatus running_status;

/* Release all notes marked as sustained in the desciptor list.
 * Return true of something actually was sent.
 */
bool release_sustained_notes(void)
{
  byte note;
  bool something_sent = false;

  for (note = 0; note < 128; note++) {
    if (note_descriptors[note] & DESC_SUSTAIN) {
      byte new_status = NOTE_ON | (note_descriptors[note] & DESC_CHANNEL_MASK);
      // Todo: fix saved note off velocity
      running_status.send(new_status);
      serial_write(apply_note_off_transpose(note, true));
      serial_write(0); /* vel = 0 >= note off */
      something_sent = true;
    } else
      /* When releasing pedal, the sostenuto bit for notes that are still playing because the
       * corresponding key is still down also needs to be cleared, or else they will continue
       * to be sostenutod even if the pedal is up.
       * We to this unconditionally even if sostenuto mode is not active, in order to clean up
       * the list in the event of a mode change. */
      sostenuto_held.clear(note);
  }
  return something_sent;
}

/* Set all played notes to sustain.
 */
void set_played_to_sustain(void)
{
  byte note;

  for (note = 0; note < 128; note++) {
    if (note_descriptors[note])
      sostenuto_held.set(note);
  }
}


#define SETTINGS_BASE_NOTE 36 /* standard bottom note of 5 and 4 octave keyboard */

/* We map two octaves of keys to 14 'function' keys (white), and 10 'value'
 * keys (black), the latter numbered 0..9. */

#define FUNCTION_BIT 128

PROGMEM const byte settings_keymap[] = {
  /* First octave: white keys are function #0..6, black keys are values #0..4 */
  FUNCTION_BIT | 0, /* C - function 0 */
  0, /* C# - value 0 */
  FUNCTION_BIT | 1, /* D - function 1 */
  1, /* D# - value 1 */
  FUNCTION_BIT | 2, /* E - function 2 */
  FUNCTION_BIT | 3, /* F - function 3 */
  2, /* F# - value 2 */
  FUNCTION_BIT | 4, /* G - function 4 */
  3, /* G# - value 3 */
  FUNCTION_BIT | 5, /* A - function 5 */
  4, /* A# - value 4 */
  FUNCTION_BIT | 6, /* B - function 6 */
  /* Second octave: white keys are function #7..13, black keys are values #5..9 */
  FUNCTION_BIT | 7, /* C - function 7 */
  5, /* C# - value 5 */
  FUNCTION_BIT | 8, /* D - function 8 */
  6, /* D# - value 6 */
  FUNCTION_BIT | 9, /* E - function 9 */
  FUNCTION_BIT | 10, /* F - function 10 */
  7, /* F# - value 7 */
  FUNCTION_BIT | 11, /* G - function 11 */
  8, /* G# - value 8 */
  FUNCTION_BIT | 12, /* A - function 12 */
  9, /* A# - value 9 */
  FUNCTION_BIT | 13, /* B - function 13 */
};

void process_setting(byte data)
{
  static byte state = STATE_PASS;
  static byte note;

  if (data & 0x80) { /* MIDI status byte */
    byte status = data & 0xf0;

    if (status == NOTE_ON)
      state = STATE_NOTE_ON_NOTE;
    else
      state = STATE_PASS;
  } else { /* MIDI data byte */
      if (state == STATE_NOTE_ON_NOTE) {
      note = data;
      state = STATE_NOTE_ON_VEL;
    } else if (state == STATE_NOTE_ON_VEL) {
      if (data) { /* note on */
        byte funcval = -1;

        note -= SETTINGS_BASE_NOTE;
        if (note < sizeof(settings_keymap) / sizeof(settings_keymap[0])) {
          funcval = pgm_read_byte(&settings_keymap[note]);
          if (funcval & FUNCTION_BIT) { /* function */
            funcval &= ~FUNCTION_BIT;
            if (funcval < sizeof(settings_screens) / sizeof(settings_screens[0]))
              settings_screen = funcval;
          } else { /* value */
            const struct screen_def *screen = &settings_screens[settings_screen];
            void (*val_func)(const struct screen_def *, byte) = pgm_read_ptr(&screen->val_func);
            if (val_func) val_func(screen, funcval);
          }
#ifdef UI_DISPLAY
          display_screen(&settings_screens[settings_screen]);
#endif
        }
      }
      state = STATE_NOTE_ON_NOTE;
    }
  }
}

void process_midi(byte data)
{
  static byte channel, prev_channel = -1; /* last received channel, and one for previous note on */
  static byte prev_octave_encoded = -1; /* transposition for previous note on */
  static byte state = STATE_PASS;
  static byte note; /* saved note across note on/off message reception, in !low_latency_mode */
  static byte addr; /* saved control change address */
  static bool fresh_status_byte = false; /* no current input running status */
  static struct note queued_note_off = { 0 };
#ifdef PROCESS_CC
  static bool skipping_cc = false;
#endif
  static bool sustaining = false;
  bool skip = false;
  bool trigger_queued_note_off = false;

  /* Normally data received is echoed just after the if clause, unless for some
   * reason processing needs to be delayed (note off messages, note on messages in
   * normal latency mode, control change messages when PROCESS_CC is set), in which
   * case the normal echoing of data is skipped and deferred until some later pass,
   * and the queued-up status and potentially first data byte are echoed then, followed by
   * the normal echoing of the final data byte.
   */
  if (data & 0x80) { /* MIDI status byte */
    byte status = data & 0xf0;

    channel = data & 0x0f;

    if (channelize) {
      channel = channelize - 1;
      /* We echo and process data, so recreate with new channel */
      data = status | channel;
    }

#ifdef MIRROR_INCOMING_RUNNING_STATUS
    /* If we get a status byte for a channel message, then running status is
     * not being employed for the currently received message. */
    if (status < 240)
      fresh_status_byte = true;
#endif

#if 0 // ?needed
    /* If we get a status byte for a channel message, then running status is
     * not being employed for the currently received message.
     * We need to remember this for certain message types (e.g. CC No Running Status
     * mode). */
    if (status < 240)
      input_running_status = false;
#endif

    /* Track note on/off status. All other channel messages just get sent
     * through.
     */
    if (status == NOTE_ON) {
#ifdef RELEASE_ALL_NOTES_WHEN_CHANNEL_OR_OCTAVE_CHANGED
      if (MODE_SET(MODE_SPE_RELEASE_ALL_WHEN_CHANNEL_CHANGED) &&
          (channel != prev_channel || octave_encoded != prev_octave_encoded)) {
        /* We got a note on on a different channel or which will have a different tranposition
         * than the previous one received. If there are sustained notes, release them, as we
         * won't be able to keep track of them if a key is pressed corresponding to an
         * already sustained note, and we want a consistent behavior so the release of sustained
         * notes happen seemingly haphazardly (i.e. depending on the actual notes depressed).
         */
        release_sustained_notes();
        /* Signal to subsequent note number processing that notes have been released and thus
         * don't need to be released again; in particular, that no additional status byte be
         * sent after the note offs.
         */
      }
#endif
#ifdef DONT_SUSTAIN_WHEN_NOTE_CHANNEL_CHANGED
      if (MODE_SET(MODE_SPE_PEDAL_UP_WHEN_CHANNEL_CHANGED) && prev_channel != channel)
        sustaining = false;
#endif
      prev_channel = channel;
      prev_octave_encoded = octave_encoded;
      if (!low_latency_mode)
        skip = true;
      state = STATE_NOTE_ON_NOTE; /* next byte will be note */
    } else if (status == NOTE_OFF) {
      low_latency_mode = true; /* if we recieve a real note off, then we can go to low latency mode */
      state = STATE_NOTE_OFF_NOTE; /* next byte will be note */
#ifdef HANDLE_CHANNEL
      /* When receiving note off, defer message until we get the note number. That way
       * we can adjust the channel number to match the note on channel. This adds a 320 us delay
       * to note off messages, compared to all other messages, but we can assume that note off
       * messages are by way of their function not as critical as note off messages.
       */
      skip = true; /* defer sending status byte until we have received note number. */
#endif
      /* If sostenuto mode is active, then we can't echo the note off as yet, but need to wait
       * until we know which note it is. */
      if (sustaining || MODE_SET(MODE_SPE_SOSTENUTO))
        skip == true;
    }
#ifdef PROCESS_CC
      else if (status == CONTROL_CHANGE) {
      state = STATE_CC_ADDR;
      skip = true;
    }
#endif
    /* All other (channel and system) messages pass through */
    else if (status <= REALTIME)
      state = STATE_PASS;
  } else { /* MIDI data byte */
    switch (state) {
      case STATE_NOTE_ON_NOTE:
#ifdef RELEASE_ALL_NOTES_WHEN_CHANNEL_OR_OCTAVE_CHANGED
        if (MODE_SET(MODE_SPE_RELEASE_ALL_WHEN_CHANNEL_CHANGED) &&
            octave_encoded != prev_octave_encoded) {
          /* This is where we handle a changed octave while experiencing incoming running status.
           * If we were not experiencing incoming running status, this would have been handled
           * higher up, when the status byte was received (in which case we also need to handle
           * the case of the channel being changed; this cannot happen with running status).
           * If we get here, thus, we are experiencing running status, and no byte for the currently
           * incoming message has yet been sent; thus we can send all the note offs for the
           * sustained notes now in order to release them. However, we then have to reinstate
           * the potentially different status byte than the one sent for the note offs, but we
           * do that anyway further down, so no special handling is needed for that case here.
           */
          release_sustained_notes();
          prev_octave_encoded = octave_encoded;
        }
#endif
        note = data; /* also save for later */
        if (low_latency_mode) {
          /* Here's the rub: not only might we have sent note offs on a different channel than
           * is currently being received, after sending a previous note on we might have transmitted
           * a note off on a different channel (a queued_note_off). We can't unconditionally send
           * a note on status here, because if the currently ongoing message did include a note on,
           * it will be sent twice, The running status processing would technically take care of this,
           * but that fails if our running status timeout hits at precisely this time. Therefore, we
           * have the slightly odd situation here that if the last sent status is not the same as
           * what would be used for the currently ongoing message, we send a new status message,
           * which is similar to how running status works, except that this also works if running
           * status for whatever reason (most likely, the running status timeout hit) is disabled.
           * On the face of it, it looks like this would allow us to send two status bytes back
           * to back, but the only way the status could be different is if running status is
           * employed and the previously sent message was a queued note off; if the ongoing message
           * was the last status sent, the channel will be unchanged and consequently the status
           * byte will be unchanged as well, and no extra status byte will be sent.
           * (Case in point: Zynthian upon receiving a double note on status bytes will behave oddly,
           * confusing subequent note on and off messages.)
           */
          byte new_status = NOTE_ON | channel;
          if (running_status.last_status() != new_status)
            running_status.send(new_status);
          /* If we find that there is already an active note for the source note number,
           * send a note off message. This means that the note is sustained, as otherwise when we
           * would have received a note off for it, the entry in the list would have been cleared
           * (or the source keyboard sent two note ons without an intermediate note off).
           */
          if (note_descriptors[note]) {
            bool same_chtr =
              ((note_descriptors[note] & DESC_CHANNEL_MASK) == channel &&
              ((note_descriptors[note] & DESC_OCTAVE_MASK) >> DESC_OCTAVE_SHIFT) == octave_encoded);
            /* If it's the same channel and transposition, it could potentially be the same
             * note number, so need to send any note off at this point prior to sending the
             * note on, to avoid the note getting chopped off if it's the same note number.
             * Conversely, if it's on another channel, we can't send it here, as we've already
             * passed through the note on status byte for the note on channel in question.
             * In any case, we need to apply note off transposition before registering
             * the new note on, so that needs to happen here as well. */
             if (!same_chtr) {
              queued_note_off.status = NOTE_ON | (note_descriptors[note] & DESC_CHANNEL_MASK);
              queued_note_off.note = apply_note_off_transpose(note, true);
             }
#ifdef SEND_NOTE_OFF_FOR_EACH_SUSTAINED_NOTE
               else if (MODE_SET(MODE_SPE_AVOID_STACKUP)) {
              /* Same channel and transpose. Send a note off immediately to avoid the voices
               * stacking up, which is nice, but can cause synths to run out of voices (depending
               * on how the voice allocation algorithm is implemented). */
               serial_write(apply_note_off_transpose(note, false));
               serial_write(0); /* note off */
               /* we utilize running status here, for the note on currently in progress */
             }
#endif
          }
          data = apply_note_on_transpose(note, channel);
        } else { /* !low_latency_mode */
          skip = true; /* don't send note number now */
        }
        break;
      case STATE_NOTE_ON_VEL:
        if (low_latency_mode) {
          /* If we get vel == 0, leave low latency mode, but we can't do anything about the currently
           * outgoing message, as we have already sent the note number, so just ride with it
           * (hope the transposition has not been changed since corresponding note on was sent).
           */
          if (data == 0) {
            low_latency_mode = false;
            /* Since we thought we were in low latency mode up till now, but it turns out we're
             * actually receiving a note off, we just run with it: send the message through.
             * This means that the first note off when entering low latency mode cannot be
             * held by the sustain pedal emulation function.
             * All that remains to do here is mark it as released in the note descriptor list. */
            note_descriptors[note] = 0;
            queued_note_off.status = 0;
            /* There is a slight issue here in that in the transition to !low_latency_mode, we've
             * already sent a note off (if SEND_NOTE_OFF_FOR_EACH_SUSTAINED_NOTE was set),
             * and we're on our way to send what we thought (due to being in low latency mode)
             * was a note on but which turned out to be a note off. This means that:
             * a) The first note off which causes the switch to !low_latency_mode will be doubled.
             * b) That note can not be sustained in sustain pedal emulation mode.
             * This is essentially not fixable, but should in practice not cause any ill effects,
             * so we'll leave it as it is.
             */
          }
        } else { /* !low_latency_mode */
          if (data) { /* true note on */
            if (note_descriptors[note]) {
              bool same_chtr =
                ((note_descriptors[note] & DESC_CHANNEL_MASK) == channel &&
                ((note_descriptors[note] & DESC_OCTAVE_MASK) >> DESC_OCTAVE_SHIFT) == octave_encoded);
              /* If it's the same channel and transposition, it could potentially be the same
               * note number, so need to send any note off at this point prior to sending the
               * note on, to avoid the note getting chopped off if it's the same note number.
               * Conversely, if it's on another channel, we can't send it here, as we've already
               * passed through the note on status byte for the note on channel in question.
               * In any case, we need to apply note off transposition before registering
               * the new note on, so that needs to happen here as well.
               * Even in low latency mode, we queue this up if we can, so that the ongoing
               * note on gets priority. */
              if (!same_chtr) {
                queued_note_off.status = NOTE_ON | (note_descriptors[note] & DESC_CHANNEL_MASK);
                queued_note_off.note = apply_note_off_transpose(note, true);
              }
#ifdef SEND_NOTE_OFF_FOR_EACH_SUSTAINED_NOTE
                else {
                /* Same channel and transpose. Send a note off immediately to avoid the voices
                 * stacking up, which is nice, but can cause synths to run out of voices (depending
                 * on how the voice allocation algorithm is implemented).
                 * Todo: Have this as a switchable option, also as it adds latency.
                 */
                 running_status.send(NOTE_ON | channel, fresh_status_byte);
                 serial_write(apply_note_off_transpose(note, false));
                 serial_write(0); /* note off */
                 /* we utilize running status here, for the note on currentl in progress */
              }
#endif
            }
            /* Note on: Time to send note on message: send status + note no here, vel further on */
            running_status.send(NOTE_ON | channel, fresh_status_byte);
            serial_write(apply_note_on_transpose(note, channel));
          } else { /* note off */
            /* In sostenuto mode, only skip note offs that are not already sustaining */
            if (sustaining &&
                (!MODE_SET(MODE_SPE_SOSTENUTO)) || sostenuto_held.is_set(note)) {
              skip = true;
              note_descriptors[note] |= DESC_SUSTAIN; /* indicate note off received and skipped */
            } else {
              byte new_status = NOTE_ON | (note_descriptors[data] ?
                                           (note_descriptors[data] & DESC_CHANNEL_MASK) :
                                           channel);
              running_status.send(new_status, fresh_status_byte);
              serial_write(apply_note_off_transpose(note, true));
            }
          }
        }
        fresh_status_byte = false; /* next non-status byte received will employ running status */
        break;
      case STATE_NOTE_OFF_NOTE:
        note = data; /* also for later */
        if (sustaining &&
            (!MODE_SET(MODE_SPE_SOSTENUTO)) || sostenuto_held.is_set(note)) {
          skip = true;
          note_descriptors[note] |= DESC_SUSTAIN; /* indicate note off received and skipped */
        } else {
#ifdef HANDLE_CHANNEL
          /* Now it's time to send our note off status. */
          {
            byte new_status = NOTE_OFF | (note_descriptors[note] ?
                                          (note_descriptors[note] & DESC_CHANNEL_MASK) :
                                          channel);
             /* We honor running status here, as we potentially need to insert a status byte, if the
              * incoming stream employs running status while the transpose is being changed, so we
              * want to minimize the number of inserted bytes added.
              */
            running_status.send(new_status, fresh_status_byte);
          }
#endif
          data = apply_note_off_transpose(note, true);
        }
        break;
      case STATE_NOTE_OFF_VEL:
        if (sustaining &&
            (!MODE_SET(MODE_SPE_SOSTENUTO)) || sostenuto_held.is_set(note))
          skip = true;
        else
          fresh_status_byte = false; /* next non-status will employ running status */
        break;
#ifdef PROCESS_CC
      case STATE_CC_ADDR:
        addr = data;
        skipping_cc = false;
#ifdef SKIP_CC
        skipping_cc |= (!!data && !!strchr(SKIP_CC, data));
#endif
#ifdef EMULATE_SUSTAIN_PEDAL
        if (MODE_SET(MODE_SPE))
          skipping_cc |= (data == CC_SUSTAIN_PEDAL);
#endif
        if (skipping_cc)
          skip = true;
        else {
          /* We've got CC + address, so send it on */
          /* Value will be sent when we get it */
          byte new_status = CONTROL_CHANGE | channel;
          running_status.send(new_status, fresh_status_byte ||
                                          data == CC_SUSTAIN_PEDAL && MODE_SET(MODE_CC_PED_NRS));
        }
        break;
      case STATE_CC_VAL:
        if (skipping_cc)
          skip = true;
#ifdef EMULATE_SUSTAIN_PEDAL
        if (MODE_SET(MODE_SPE) && addr == CC_SUSTAIN_PEDAL) {
          if (data) {
            sustaining = true;
            if (MODE_SET(MODE_SPE_SOSTENUTO))
              set_played_to_sustain();
          } else {
            /* pedal released. Time to released all sustained notes. */
            sustaining = false;
            release_sustained_notes();
          }
        }
#endif
        fresh_status_byte = false; /* set for next CC_ADDR state */
        break;
#endif
      default: /* Do nothing, just echo byte received */
        break;
    }
    /* Manage state transitions */
    switch (state) {
      case STATE_NOTE_ON_NOTE: state = STATE_NOTE_ON_VEL; break;
      case STATE_NOTE_ON_VEL: state = STATE_NOTE_ON_NOTE;
                              /* If a note off has been queued, send it after the velocity byte
                               * has been sent. */
                              if (queued_note_off.status)
                                trigger_queued_note_off = true;
                              break;
      case STATE_NOTE_OFF_NOTE: state = STATE_NOTE_OFF_VEL; break;
      case STATE_NOTE_OFF_VEL: state = STATE_NOTE_OFF_NOTE; break;
      case STATE_CC_ADDR: state = STATE_CC_VAL; break;
      case STATE_CC_VAL: state = STATE_CC_ADDR; break;
      default: break;
    }
  }
  if (!skip) {
    if ((data & 0x80) && data < 248) { /* Status messages that are not realtime */
      if (data < 240) {
        running_status.send(data, fresh_status_byte);
      } else {
        serial_write(data);
        running_status.clear();
      }
    } else {
      serial_write(data); /* realtime or data byte */
    }
  }
  if (trigger_queued_note_off) {
    /* When we end up here, it's because we've just sent a note on, and need to send a note off
     * for the note previously occupying the note descriptor, to avoid it hanging. */
    running_status.send(queued_note_off.status);
    serial_write(queued_note_off.note);
    serial_write(0); /* => note off */
    queued_note_off.status = 0;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(31250);
#ifdef UI_DISPLAY
  disp.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS);
  /* clear screen by writing maximum size spaces */
  disp.printDirect(0, 0, strcpy_P(strbuf, triple_space_s), 8);
#endif

  pinMode(TRANSPOSE_PIN_0, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_1, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_2, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_3, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_4, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_5, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  running_status_time = led_flash_time = micros();
  running_status.clear(); /* outgoing running status */
}

void loop() {
  byte new_octave_encoded;

  now = micros();

  // put your main code here, to run repeatedly:
  /* Turn led off at end of flash period */
  if (now - led_flash_time > LED_FLASH_US)
    digitalWrite(LED_BUILTIN, LOW ^ !low_latency_mode);

#ifdef RUNNING_STATUS_TIMEOUT_US
  if (now - running_status_time > RUNNING_STATUS_TIMEOUT_US)
  {
    running_status.clear(); /* force next message status byte to be sent */
    running_status_time = now; /* reset timer */
  }
#endif

  new_octave_encoded = read_octave_switch();
#ifdef ENABLE_PARAMETER_SETTING
  if (new_octave_encoded > 1) {
    octave_encoded = new_octave_encoded;
    setting_parameters = false;
  } else {
    setting_parameters = true;
  }
#else
  octave_encoded = new_octave_encoded;
#endif
  if (octave_encoded != octave_prev || setting_parameters != setting_parameters_prev) {

    /* Courtesy calculation - so we don't need to do it for each note on */
    transpose = octave_encoded * 12 - OCTAVE_OFFSET * 12;
#ifdef UI_DISPLAY
    if (setting_parameters)
      display_screen(&settings_screens[settings_screen]);
    else
      display_screen(&octave_screen);
#endif
    octave_prev = octave_encoded;
    setting_parameters_prev = setting_parameters;
  }

  if (Serial.available()) {
    /* Flash led = turn it on here, and set timeout */
    digitalWrite(LED_BUILTIN, HIGH ^ !low_latency_mode);
    led_flash_time = now;

    byte data = Serial.read();
    if (setting_parameters)
      process_setting(data);
    else {
      process_midi(data);
#ifdef MIDIDUMP
      if (dump_mode == DUMPMODE_IN) {
        dump(data);
        dump_time_us = now;
      }
#endif
    }
  }
#ifdef MIDIDUMP
  /* print item in MIDI dump buffer if no data arrived for a while */
  /* Caveat: This cold go wrong if the source never shuts off, for instance if is
   * transmitting MIDI clock. */
  if (!setting_parameters && dumpbuf_fill > 0 && now - dump_time_us >= DUMP_TIMEOUT_US)
    display_dump(false);
#endif
}
