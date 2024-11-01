# Low latency MIDI octave transposer

# Introduction

This is the source for an Arduino based low latency octave switching device,
intended for use with keyboards who lack or have hard-to-access octave
switching facilities, such as the Synthesizers.com QKB series. The octave
switching is achieved using a rotary selector or pushbuttons, shorting
the appropriate input pin of the Arduino to ground.

MIDI in and out is connected to the serial ports on the Arduino, pins
D0 and D1, respectively, on an Arduino UNO.

# Octaver

The basic functionality is a fast (low latency - 320 µs) octave transposer,
which can shift the octave of the notes in the MIDI stream by
-3 (only when parameter setting is not enabled build time in the code),
-2, -1, 0, +1 or +2 octaves.

The lowest latency is achieved with keyboards which output MIDI Note Off
messages for note off events, rather than MIDI Note on messages with the
velocity set to 0. In the former case, the latency through the device is
a mere 320 µs. This is known as low latency mode. In the latter case, a further
320 µs of latency is added. This is known as normal mode. The device switches
between the two automatically depending on what MIDI messages are present
in the incoming MIDI stream. Normal mode is indicated by the MIDI LED
(connected to the LED_BUILTIN (D13) pin on the Arduino) being inverted, i.e.
normally on, and flashes off when MIDI data is seen. In low latency mode,
the MIDI LED is normally off, and flashes on for MIDI data.

A salient feature of the octaver is to handle the cases of either the source
MIDI channel being changed between the note on and subsequent note off
message, or the octave being changed under these conditions. The
octaver saves the original transposition and MIDI channel of the note on,
and transmits the note off on the appropriate MIDI channel and with the
appropriate tranposition. This avoids any hung notes, and also allows
for creative use of the octaver, for instance by playing a low note, then
switching the transposer to a higher octave and play high notes while holding
the low note.

Note that MIDI key pressure messages are currently not transposed.

# Other features

Additionally, there are other optional features which may be enabled:
- (Absolute) Channelize. Force all incoming messages to a specific
  MIDI channel.
- Relative Channelize. Shift the channel of incoming messages by
  an amount 1..15 . The channel numbers are wrapped if they go above
  16, e.g. an incoming message on channel 3 will be output on channel 2
  if the relative channelize is set to 15.
- Sustain pedal emulation (SPE), for emulating the sustain pedal (MIDI
  Control Change #64) using ordinary note on and off messages.
- As a sub mode to SPE, Sostenuto may be employed rather than Sustain.
  In Sostenuto mode, only notes which are being played when the pedal
  is depressed are sustained; subsequent notes play as if the pedal were
  not pressed.
- Skip-CC mode, whereby selected MIDI Control Change messages can be
  removed from the MIDI stream passing through the device. This feature
  is normally disabled.
- Sustain Pedal No Running Status mode. This is intended to fix a problem
  with the Waldorf Blofeld, which needs the status byte to be sent for
  each sustain pedal message in order not to miss it. Normally in MIDI,
  the status byte does not need to be repeated if the next message in
  the stream has the same status byte, however, the Blofeld can't handle
  this when it comes to sustain pedal messages.

### SPE sub modes

SPE mode further has the following sub modes:
- SPE Max 1: When the sustain pedal is pressed, and the same key is pressed
  over and again, send a note off prior to each repeated note on to avoid
  the voices stacking up in the receiving instrument.
- SPE R All: Normally, when the sustain pedal is down, and notes are
  received on a new channel or with a new transpose setting, old notes
  are sustained as far as possible. However, SPE R All mode, whenever a note
  is received on a different channel or with a different transposition
  than the previous note, all previously held notes are released.
- SPE P U Ch: Normally, whenever the source channel is changed, the
  sustain pedal state is retained, so that new notes played on the new channel
  are also held. However, in SPE P U Ch mode, whenever a note is received
  on a different channel, the pedal is set to the up (released) state, meaning
  that new notes are not sustained, however, existing held notes are not
  released, and will be released first when the pedal pressed and released
  again.

## Build time configuration

The octaver can be operated as a pure octaver box, in which case the octave
shift can be set from -3 to +2 . Optionally, parameter setting can be
enabled; in which case the lowest octave position is replaced by entry into
parameter setting mode; consequently the octave shift range is reduced to
-2 to +2.

## OLED display

An optional 128x64 OLED display can be connected via I2C using pins A4 (SDA)
and A5 (SCL), for indicating the current octave setting and other status, as
well as current parameter settings.

## MIDI dump

A MIDI dump mode is available, which dumps the latest 12 bytes received or
sent (configurable) in the right hand part of the display.

Note that while uploading new sketches, seemingly garbage will be emitted
on the MIDI output which can cause all sorts of results on potentially
connected instruments. Furthermore, any incoming MIDI will disrupt the
uploading process. The easiest solution to these issues is to disconnect
the MIDI IN and OUT connectors while uploading sketches to the device.

## Modwheel

Not enabled by default, the Modwheel support enables the value output from a
potentiometer connected to the appropriate pin (default A0, 0..5V) on the
Arduino to be inserted into the MIDI stream as MIDI Control Change #1 (mod
wheel). Enable by #defining MODWHEEL.

## Dependencies

Unintuitively, the code is not dependent on the Arduino MIDI library.
Because of the byte-level manipulation of MIDI data, due in part to the
minimization of throughput latency, the MIDI library is not suitable.

For the display, in order to permit printing out MIDI byte values
in real time without adding undue delay to the MIDI stream, parts
of the Adafruit_SSD1306 library have been refactored into a fast
(albeit limited) display printing class. There is still a dependency
on the underlying Adafruit_GFX library in the form of the font
used for printouts, and there is also a formal (due to licensing
requirements) dependency on the Adafruit_SSD1306 because of the
re-use of the code, although there are no actual functional dependencies
on either of these libraries.

## Hardware

The Arduino pins are used as follows:

D0 -  MIDI IN  
D1 -  MIDI OUT  
D2 -  Input: Ground for Octave -3 / Settings mode  
D3 -  Input: Ground for Octave -2  
D4 -  Input: Ground for Octave -1  
D5 -  Input: Ground for Octave 0  
D6 -  Input: Ground for Octave +1  
D7 -  Input: Ground for Octave +2  
D13 - MIDI activity LED anode (active high)  
A4 -  OLED display SDA  
A5 -  OLED display SCL  
