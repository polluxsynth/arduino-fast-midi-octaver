#include <string.h>
/* For OLED graphics: */
#include <Adafruit_SSD1306.h>

/*
 * Fast (low latency - 320 us) note transpose filter.
 */

/* Released under GPLv2. See LICENSE for details. */

/* Features/limitations:
   - In low latency mode, can only handle keyboards where note off is sent with status 128.
   - When 144 with velocity 0 is received, switches to normal latency mode, where complete note on/off
     message must be received before sending it on (latency 960 us).
     - The first 144 note off received in low latency mode will not be shifted by the transpose memory.
       This is unlikely to be a problem in practice, as is the first note received.
   - When status 128 is received, switches to low latency mode.
   - Status LED (D13) flashes 10 ms on incoming data. The status LED is inverted in normal mode
     (i.e. normally on, and goes dark 10 ms on incoming data).
   - In low latency mode, does not remember original channel when note off received,
     unless HANDLE_CHANNEL is set. This adds a further 320 us.
   - Mimics running status of source.
   - When SKIP_CC is set, skips all CC 22, 23 and 24 messages. This feature adds
     320 us of latency to all passing CC messages.
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

/* Convert sustain pedal to delayed note offs. */
#define SUSTAIN_TO_NOTE_OFF

#if defined(SKIP_CC) || defined(SUSTAIN_TO_NOTE_OFF)
#define PROCESS_CC
#endif

#undef MIRROR_INCOMING_RUNNING_STATUS
/* Every second, reset running status to avoid long periods without status on output */
#ifndef MIRROR_INCOMING_RUNNING_STATUS
#define RUNNING_STATUS_TIMEOUT_US 1000000
#endif

#define UI_DISPLAY

#define OCTAVE_OFFSET 4 /* e.g. octave -3 => encoded octave value is 1 */

/* Digital pins 0 and 1 are used for (MIDI) serial communication, so use digital I/O
   2 and upwards for the transpose switch input.
*/
#define TRANSPOSE_PIN_0        2
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
#define BODY_X 2
#define BODY_Y 24
#define BODY_TEXTSIZE 4

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SSD1306_I2C_ADDRESS 0x3c
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

#endif

byte transpose = 0;
byte octave_encoded = OCTAVE_OFFSET;
byte octave_prev = 0;
bool low_latency_mode = true;

long int led_flash_time;
long int running_status_time;

#define DESC_CHANNEL_MASK 0x0f
#define DESC_OCTAVE_MASK 0x70 /* encoded octave */
#define DESC_OCTAVE_SHIFT 4
#define DESC_SUSTAIN 0x80 /* sustained by sustain pedal */

byte note_descriptors[128] = { 0 };

byte read_octave(void)
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

#ifdef UI_DISPLAY
void display_octave(byte octave)
{
  char octave_str[3] = "  ";
  byte disp_octave = octave;

  if (octave < OCTAVE_OFFSET) {
    octave_str[0] = '-';
    disp_octave = OCTAVE_OFFSET - octave; /* always positive */
  } else if (octave > OCTAVE_OFFSET) {
    octave_str[0] = '+';
    disp_octave = octave - OCTAVE_OFFSET;
  } else
    disp_octave = 0;
  octave_str[1] = disp_octave + '0';

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(HEADER_TEXTSIZE);
  display.setCursor(HEADER_X, HEADER_Y);
  display.println("Octave");

  display.setTextSize(BODY_TEXTSIZE);
  display.setCursor(BODY_X, BODY_Y);
  display.println(octave_str);

  display.display();
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

byte apply_note_off_transpose(byte note)
{
  byte ret = note;

  if (!note_descriptors[note]) /* hasn't been set, don't transpose */
    return note;

  ret += ((note_descriptors[note] & DESC_OCTAVE_MASK) >> DESC_OCTAVE_SHIFT) * 12 - OCTAVE_OFFSET * 12;

  /* Mark the note as released */
  note_descriptors[note] = 0;

  return ret;
}

class RunningStatus {

public:
  /* Constructor */
  RunningStatus(void)
  {
    m_running_status = 0;
    m_fresh_status_byte = 0;
  }

  /* Send status byte, honoring running status, unless fresh_status_byte is set.
   * Return new running status (may be unchanged).
   */
  void send(byte status, bool fresh_status_byte)
  {
    if (status != m_running_status || fresh_status_byte) {
      m_running_status = status;
      Serial.write(status);
    }
  }

  void clear(void)
  {
    m_running_status = 0;
  }

private:
  byte m_running_status;
  bool m_fresh_status_byte;
};

RunningStatus running_status;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(31250);
#ifdef UI_DISPLAY
  display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS);
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
  long int now = micros();

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

  octave_encoded = read_octave();
  if (octave_encoded != octave_prev) {
    /* Courtesy calculation - so we don't need to do it for each note on */
    transpose = octave_encoded * 12 - OCTAVE_OFFSET * 12;
#ifdef UI_DISPLAY
    display_octave(octave_encoded);
#endif
    octave_prev = octave_encoded;
  }
  /* Normally data received is echoed at the very end of the if clause, unless for some
   * reason processing needs to be delayed (note off messages, note on messages in
   * normal latency mode, control change messages when PROCESS_CC is set), in which
   * case the normal echoing of data is skipped and deferred until some later pass,
   * and the queued-up status and potentially first data byte are echoed then, followed by
   * the normal echoing of the final data byte.
   */
  if (Serial.available()) {
    static byte channel; /* last received channel */
    static byte state = STATE_PASS;
    static byte note; /* saved note across note on/off message reception, in !low_latency_mode */
    static byte addr; /* saved control change address */
    static bool fresh_status_byte = false; /* no current input running status */
#ifdef PROCESS_CC
    static bool skipping_cc = false;
#endif
    static bool sustaining = false;
    byte data = Serial.read();
    bool skip = false;

    /* Flash led = turn it on here, and set timeout */
    digitalWrite(LED_BUILTIN, HIGH ^ !low_latency_mode);
    led_flash_time = now;

    if (data & 0x80) { /* MIDI status byte */
      byte status = data & 0xf0;

      channel = data & 0x0f;
#ifdef MIRROR_INCOMING_RUNNING_STATUS
      fresh_status_byte = true;
#endif

      /* Track note on/off status. All other channel messages just get sent
         through.
      */
      if (status == NOTE_ON) {
        if (!low_latency_mode)
          skip = true;
        state = STATE_NOTE_ON_NOTE; /* next byte will be note */
      } else if (status == NOTE_OFF) {
        low_latency_mode = true; /* if we recieve a real note off, then we can go to low latency mode */
        state = STATE_NOTE_OFF_NOTE; /* next byte will be note */
#ifdef HANDLE_CHANNEL
        /* When receiving note off, defer message until we get the note number. That way
           we can adjust the channel number to match the note on channel. This adds a 320 us delay
           to note off messages, compared to all other messages, but we can assume that note off
           messages are by way of their function not as critical as note off messages.
        */
        skip = true; /* defer sending status byte until we have received note number. */
#endif
        if (sustaining)
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
          if (low_latency_mode) {
            data = apply_note_on_transpose(data, channel);
          } else {
            note = data; /* save for later */
            skip = true; /* don't send note number now */
          }
          break;
        case STATE_NOTE_ON_VEL:
          if (low_latency_mode) {
            /* If we get vel == 0, leave low latency mode, but we can't do anything about the currently
               outgoing message, as we have already sent the note number, so just ride with it
               (hope the transposition has not been changed since corresponding note on was sent).
            */
            if (data == 0) {
              low_latency_mode = false;
              if (sustaining) {
                skip = true;
                note_descriptors[data] |= DESC_SUSTAIN;
              }
            }
          } else { /* !low_latency_mode */
            if (data) {
              /* Note on: Time to send note on message: send status + note no here, vel further on */
              byte new_status = NOTE_ON | channel;
              running_status.send(new_status, fresh_status_byte);
              Serial.write(apply_note_on_transpose(note, channel));
            } else {
              if (sustaining) {
                skip = true;
                note_descriptors[data] |= DESC_SUSTAIN; /* indicate note off received and skipped */
              } else {
                /* Note off: send as note on w/ vel = 0 */
                byte new_status = NOTE_ON | (note_descriptors[data] ?
                                             (note_descriptors[data] & DESC_CHANNEL_MASK) :
                                             channel);
                running_status.send(new_status, fresh_status_byte);
                Serial.write(apply_note_off_transpose(note));
              }
            }
          }
          fresh_status_byte = false; /* next non-status byte received will employ running status */
          break;
        case STATE_NOTE_OFF_NOTE:
          if (sustaining) {
            skip = true;
            note_descriptors[data] |= DESC_SUSTAIN; /* indicate note off received and skipped */
          } else {
#ifdef HANDLE_CHANNEL
            /* Now it's time to send our note off status. */
            {
              byte new_status = NOTE_OFF | (note_descriptors[data] ?
                                            (note_descriptors[data] & DESC_CHANNEL_MASK) :
                                            channel);
               /* We honor running status here, as we potentially need to insert a status byte, if the
                  incoming stream employs running status while the transpose is being changed, so we
                  want to minimize the number of inserted bytes added.
                */
              running_status.send(new_status, fresh_status_byte);
            }
#endif
            data = apply_note_off_transpose(data);
          }
          break;
        case STATE_NOTE_OFF_VEL:
          if (sustaining)
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
#ifdef SUSTAIN_TO_NOTE_OFF
          skipping_cc |= (data == CC_SUSTAIN_PEDAL);
#endif
          if (skipping_cc)
            skip = true;
          else {
            /* We've got CC + address, so send it on */
            /* Value will be sent when we get it */
            byte new_status = CONTROL_CHANGE | channel;
            running_status.send(new_status, fresh_status_byte);
          }
          break;
        case STATE_CC_VAL:
          if (skipping_cc)
            skip = true;
#ifdef SUSTAIN_TO_NOTE_OFF
          if (addr == CC_SUSTAIN_PEDAL) {
            if (data)
              sustaining = true;
            else {
              /* pedal released. Time to released all sustained notes. */
              byte note;
              sustaining = false;

              for (note = 0; note < 128; note++) {
                if (note_descriptors[note] & DESC_SUSTAIN) {
                  byte new_status = NOTE_ON | (note_descriptors[note] & DESC_CHANNEL_MASK);
                  // Todo: fix saved note off velocity
                  running_status.send(new_status, fresh_status_byte);
                  Serial.write(apply_note_off_transpose(note));
                  Serial.write(0); /* vel = 0 >= note off */
                }
              }
            }
          }
#endif
          break;
#endif
        default: /* Do nothing, just echo byte received */
          break;
      }
      /* Manage state transitions */
      switch (state) {
        case STATE_NOTE_ON_NOTE: state = STATE_NOTE_ON_VEL; break;
        case STATE_NOTE_ON_VEL: state = STATE_NOTE_ON_NOTE; break;
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
          Serial.write(data);
          running_status.clear();
        }
      } else {
        Serial.write(data); /* realtime or data byte */
      }
    }
  }
}
