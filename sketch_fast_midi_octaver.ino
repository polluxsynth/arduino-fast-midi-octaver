#include <MIDI.h>
#include <string.h>

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

// Create and bind the MIDI interface to the default hardware Serial port
//MIDI_CREATE_DEFAULT_INSTANCE();
MIDI_CREATE_INSTANCE(HardwareSerial, Serial, MIDI);

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

#undef MIRROR_INCOMING_RUNNING_STATUS
/* Every second, reset running status to avoid long periods without status on output */
#ifndef MIRROR_INCOMING_RUNNING_STATUS
#define RUNNING_STATUS_TIMEOUT_US 1000000
#endif

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
#define REALTIME 248 /* This and above */

byte transpose = 0;
bool low_latency_mode = true;

long int led_flash_time;
long int running_status_time;

struct note_descriptor {
  byte transpose; /* 2's complement but we don't tell anybody... */
  byte channel;
};

struct note_descriptor descriptors[128] = { 0 };

byte read_transpose(void)
{
  byte new_transpose = transpose;

  /* High-transpose priority */
  if (digitalRead(TRANSPOSE_PIN_5) == LOW)
    new_transpose = 24;
  else if (digitalRead(TRANSPOSE_PIN_4) == LOW)
    new_transpose = 12;
  else if (digitalRead(TRANSPOSE_PIN_3) == LOW)
    new_transpose = 0;
  else if (digitalRead(TRANSPOSE_PIN_2) == LOW)
    new_transpose = -12;
  else if (digitalRead(TRANSPOSE_PIN_1) == LOW)
    new_transpose = -24;
  else if (digitalRead(TRANSPOSE_PIN_0) == LOW)
    new_transpose = -36;

  return new_transpose;
}

byte apply_note_on_transpose(byte note)
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

  descriptors[note].transpose = note_transpose; /* save for subsequent note off */

  return note + note_transpose;
}

byte apply_note_off_transpose(byte note)
{
  /* Any overrange should have been taken care of at note on time, so just add the offset here */
  return note + descriptors[note].transpose;
}

/* Send status byte, honoring running status, unless fresh_status_byte is set.
 * Return new running status (may be unchanged).
 */
byte send_running(byte status, byte running_status, bool fresh_status_byte)
{
  if (status != running_status || fresh_status_byte) {
    Serial.write(status);
    return status;
  }
  return running_status;
}

void setup() {
  // put your setup code here, to run once:
  MIDI.begin(MIDI_CHANNEL_OMNI);  // Listen to all incoming messages
  pinMode(TRANSPOSE_PIN_0, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_1, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_2, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_3, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_4, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_5, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  running_status_time = led_flash_time = micros();
}

void loop() {
  long int now = micros();
  static byte running_status = 0; /* outgoing running status */

  // put your main code here, to run repeatedly:
  /* Turn led off at end of flash period */
  if (now - led_flash_time > LED_FLASH_US)
    digitalWrite(LED_BUILTIN, LOW ^ !low_latency_mode);

#ifdef RUNNING_STATUS_TIMEOUT_US
  if (now - running_status_time > RUNNING_STATUS_TIMEOUT_US)
  {
    running_status = 0; /* clear running status, forcing next message status byte to be sent */
    running_status_time = now; /* reset timer */
  }
#endif

  transpose = read_transpose();
  //MIDI.read(); /* Don't use MIDI library to parse MIDI data */
  /* Normally data received is echoed at the very end of the if clause, unless for some
   * reason processing needs to be delayed (note off messages, note on messages in
   * normal latency mode, control change messages when SKIP_CC_22 is set), in which
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
#ifdef SKIP_CC
    static bool skipping_cc = false;
#endif
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
      }
#ifdef SKIP_CC
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
            descriptors[data].channel = channel + 1; /* descriptors[].channel ==  0 => none set */
            data = apply_note_on_transpose(data);
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
            if (data == 0)
              low_latency_mode = false;
          } else { /* !low_latency_mode */
            if (data) {
              /* Note on: Time to send note on message: send status + note no here, vel further on */
              byte new_status = NOTE_ON | channel;
              if (new_status != running_status || fresh_status_byte) {
                Serial.write(new_status);
                running_status = new_status;
              }
              descriptors[note].channel = channel + 1; /* descriptors[].channel ==  0 => none set */
              Serial.write(apply_note_on_transpose(note));
            } else {
              /* Note off: send as note on w/ vel = 0 */
              byte new_status = NOTE_ON | (descriptors[data].channel ?
                                           (descriptors[data].channel - 1) :
                                           channel);
              running_status = send_running(new_status, running_status, fresh_status_byte);
              Serial.write(apply_note_off_transpose(note));
            }
          }
          fresh_status_byte = false; /* next non-status byte received will employ running status */
          break;
        case STATE_NOTE_OFF_NOTE:
#ifdef HANDLE_CHANNEL
          /* Now it's time to send our note off status. */
          {
            byte new_status = NOTE_OFF | (descriptors[data].channel ?
                                          (descriptors[data].channel - 1) :
                                          channel);
            /* We honor running status here, as we potentially need to insert a status byte, if the
               incoming stream employs running status while the transpose is being changed, so we
               want to minimize the number of inserted bytes added.
            */
            running_status = send_running(new_status, running_status, fresh_status_byte);
          }
#endif
          data = apply_note_off_transpose(data);
          break;
        case STATE_NOTE_OFF_VEL:
          fresh_status_byte = false; /* next non-status will employ running status */
          break;
#ifdef SKIP_CC
        case STATE_CC_ADDR:
          skipping_cc = (!!data && !!strchr(SKIP_CC, data));
          if (skipping_cc)
            skip = true;
          else {
            /* We've got CC + address, so send it on */
            /* Value will be sent when we get it */
            byte new_status = CONTROL_CHANGE | channel;
            running_status = send_running(new_status, running_status, fresh_status_byte);
          }
          break;
        case STATE_CC_VAL:
          if (skipping_cc)
            skip = true;
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
#if 1
    /* Running status */
    /* Check bit 7, so we can set running_status to 0 to disable it if we want, without
       the consequence being that we skip data bytes that have that value.
    */
    if ((data & 0x80) && data == running_status && !fresh_status_byte)
      skip = true;
#endif
    if (!skip) {
      Serial.write(data);
      if ((data & 0x80) && data < 248) /* Status messages that are not realtime */
        if (data < 240)
          running_status = data;
        else
          running_status = 0; /* disable running status for non-channel messages (sysex, spp etc) */
    }
  }
}
