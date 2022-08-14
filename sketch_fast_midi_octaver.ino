#include <MIDI.h>

/*
 * Fast (low latency - 320 us) note transpose filter.
 */

/* Released under GPLv2. See LICENSE for details. */

/* Limitations:
 * - Can only handle keyboards where note off is sent with status 128, not 144 and velocity 0.
 * - Does not remember original channel when note off received.
 */

// Create and bind the MIDI interface to the default hardware Serial port
//MIDI_CREATE_DEFAULT_INSTANCE();
MIDI_CREATE_INSTANCE(HardwareSerial, Serial, MIDI);

/* Define in order to remember note on channel when setting note offs.
 * This adds a further 320 us latency to note off messages.
 */
/* TODO: with this enabled, note offs received with no preceeding note on will be transmitted on
 * channel 1 as that what the channel is initialized to.
 * So, instead, store channel + 1, and use channel == 0 as an indication that there has been no
 * preceeding note on, and hence the note off should just be sent as it is.
 */
#define HANDLE_CHANNEL

/* Digital pins 0 and 1 are used for (MIDI) serial communication, so use digital I/O
 * 2 and upwards for the transpose switch input.
 */
#define TRANSPOSE_PIN_0        2
#define TRANSPOSE_PIN_1        3
#define TRANSPOSE_PIN_2        4
#define TRANSPOSE_PIN_3        5
#define TRANSPOSE_PIN_4        6

/* LED flash time */
#define LED_FLASH_US 10000

#define STATE_PASS             0
#define STATE_NOTE_ON_NOTE     1
#define STATE_NOTE_ON_VEL      2
#define STATE_NOTE_OFF_NOTE    3
#define STATE_NOTE_OFF_VEL     4

/* MIDI status bytes */
#define NOTE_OFF 128
#define NOTE_ON 144
#define REALTIME 248

byte transpose = 0;

long int led_flash_time;

struct note_descriptor {
  byte transpose; /* 2's complement but we don't tell anybody... */
  byte channel;
};

struct note_descriptor descriptors[128] = { 0 };

byte read_transpose(void)
{
  byte new_transpose = transpose;

  /* High-transpose priority */
  if (digitalRead(TRANSPOSE_PIN_4) == LOW)
    new_transpose = 24;
  else if (digitalRead(TRANSPOSE_PIN_3) == LOW)
    new_transpose = 12;
  else if (digitalRead(TRANSPOSE_PIN_2) == LOW)
    new_transpose = 0;
  else if (digitalRead(TRANSPOSE_PIN_1) == LOW)
    new_transpose = -12;
  else if (digitalRead(TRANSPOSE_PIN_0) == LOW)
    new_transpose = -24; 

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

void setup() {
  // put your setup code here, to run once:
  MIDI.begin(MIDI_CHANNEL_OMNI);  // Listen to all incoming messages
  pinMode(TRANSPOSE_PIN_0, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_1, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_2, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_3, INPUT_PULLUP);
  pinMode(TRANSPOSE_PIN_4, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  led_flash_time = micros();
}

void loop() {
  long int now = micros();
  
  // put your main code here, to run repeatedly:
  /* Turn led off at end of flash period */
  if (now - led_flash_time > LED_FLASH_US)
    digitalWrite(LED_BUILTIN, LOW);
  
  transpose = read_transpose();
  //MIDI.read(); /* Don't use MIDI library to parse MIDI data */
  if (Serial.available()) {
    static byte channel; /* last received channel */
    static byte running_status = 0; /* outgoing running status */
    static byte state = STATE_PASS;
    byte data = Serial.read();
    bool skip = false;

    /* Flash led = turn it on here, and set timeout */
    digitalWrite(LED_BUILTIN, HIGH);
    led_flash_time = now;
    
    if (data & 0x80) { /* MIDI status byte */
      byte status = data & 0xf0;      
      channel = data & 0x0f;
      
      /* Track note on/off status. All other channel messages just get sent
       * through.
       */
      if (status == NOTE_ON)
        state = STATE_NOTE_ON_NOTE; /* next byte will be note */
      else if (status == NOTE_OFF) {
        state = STATE_NOTE_OFF_NOTE; /* next byte will be note */
#ifdef HANDLE_CHANNEL      
        /* When receiving note off, defer message until we get the note number. That way
         * we can adjust the channel number to match the note on channel. This adds a 320 us delay
         * to note off messages, compared to all other messages, but we can assume that note off
         * messages are by way of their function not as critical as note off messages.
         */
        skip = true; /* defer sending status byte until we have received note number. */
#endif
      }
      /* All other (channel and system) messages pass through */
      /* Realtime messages don't alter running status however */
      else if (status <= REALTIME)
        state = STATE_PASS; 
    } else { /* MIDI data byte */
      if (state == STATE_NOTE_ON_NOTE) {
        descriptors[data].channel = channel + 1; /* descriptors[].channel ==  0 => none set */
        data = apply_note_on_transpose(data);
      } else if (state == STATE_NOTE_OFF_NOTE) {
#ifdef HANDLE_CHANNEL
        /* Now it's time to send our note off status. */
        byte new_status = NOTE_OFF | (descriptors[data].channel ?
                                        (descriptors[data].channel - 1) :
                                        channel);
        /* We honor running status here, as we potentially need to insert a status byte, if the incoming
         * stream employs running status while the transpose is being changed, so we want to minimize
         * the number of inserted bytes added. The downside is that, as implemented, running status will
         * be employed even if the input data does not employ it. However, since a limitation we
         * have is that note offs need to be sent as distinct messages, the running status period
         * will be terminated as soon as we receive a note on (or other non-note-off) message.
        */
        if (new_status != running_status) {
          Serial.write(new_status);
          running_status = new_status;
        }
#endif
        data = apply_note_off_transpose(data);
      }
      /* Track data position, so we can keep track of running status. */
      switch (state) {
        case STATE_NOTE_ON_NOTE: state = STATE_NOTE_ON_VEL; break;
        case STATE_NOTE_ON_VEL: state = STATE_NOTE_ON_NOTE; break;
        case STATE_NOTE_OFF_NOTE: state = STATE_NOTE_OFF_VEL; break;
        case STATE_NOTE_OFF_VEL: state = STATE_NOTE_OFF_NOTE; break;
        default: break;
      }
    }
#if 1
    /* Running status */
    /* Check bit 7, so we can set running_status to 0 to disable it if we want, without
     * the consequence being that we skip data bytes that have that value. 
     */
    if ((data & 0x80) && data == running_status)
       skip = 1;
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
