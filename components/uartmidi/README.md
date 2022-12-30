# UART MIDI Driver

## Introduction

The API of this UART MIDI driver has been tailored to work perfectly together with other stream based
drivers, such as:

   * BLEMIDI Driver for communication via Bluetooth: https://github.com/midibox/esp32-idf-blemidi
   * Apple MIDI Driver for communication via WIFI: https://github.com/midibox/esp32-idf-applemidi

## Pinning

The pinning is configurable via #define statements, the default pinning can be found in uartmidi.c:

   * uartmidi_port == 0 -> Rx Pin 16, Tx Pin 17
   * uartmidi_port == 1 -> Rx Pin 3, Tx Pin 1
   * uartmidi_port == 2 -> Rx Pin 3, Tx Pin 1

Note that with uartmidi_port 2 we use the same pinning like port 1, but UART0 is assigned.
Typically not a good choice, since this UART is also used for debug messages on the console!


## Connection

For traditional MIDI use following circuit:
<TODO> draw picture...

Alternatively the so called "Hairless MIDI" could be used, which connects to a USB Serial Bridge:
https://projectgus.github.io/hairless-midiserial

E.g. on a NodeMCU board, this works if uartmidi_port == 1 is used with baudrate 115200


## Usage

See the demo under ../../main/uartmidi_demo.c for general usage.


### Initialization

Launch the UART MIDI Service with:

```c
  uartmidi_init(uartmidi_receive_message_callback);
```

and:
* register a callback for incoming MIDI messages
* launch a task for sending MIDI messages


### Receiving MIDI

See callback_midi_message_received() function in uartmidi_demo.c.

API:
```c
void callback_midi_message_received(uint8_t uartmidi_port, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
```

* uartmidi_port is according to the enabled MIDI port
* midi_status contains the MIDI Status byte - running status has already been considered by the driver
* remaining_message and len: the remaining bytes (typically 2, e.g. Note or CCs, or much more on SysEx streams)
* continued_sysex_pos is >0 on a new SysEx chunk until 0xf7 will be received


### Sending MIDI

Currently no comfortable API exists, it might come later:
```c
    {
      // TODO: more comfortable packet creation via special APIs
      uint8_t message[3] = { 0x90, 0x3c, 0x7f };
      uartmidi_send_message(uartmidi_port, message, sizeof(message));
    }
```

## Using with IDF Component Manager

Simply add to your idf_component.yml
```
## IDF Component Manager Manifest File
dependencies:
  uartmidi:
    version: ">=0.0.1"
    path: components/uartmidi
    git: https://github.com/midibox/esp32-idf-uartmidi.git

```

See the documentation
[here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html)
for more informations.
