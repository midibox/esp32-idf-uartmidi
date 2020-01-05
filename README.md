# UART MIDI Driver Example

## Scope

This demo shows how to run a UART MIDI Service

Incoming data from a MIDI IN port will be loopbacked to the MIDI OUT port.

By default we use:
   * Rx Pin: 16
   * Tx Pin: 17

and send with common MIDI baudrate (31250)

The demo also shows how to select Tx Pin 1/ Rx Pin 3, which is typically connected to a UART-USB bridge, which
can be used in conjunction with "Hairless MIDI" to send MIDI data over USB: https://projectgus.github.io/hairless-midiserial


Re-usable component is located under components/uartmidi - please see the README.md for programmers there.


## Important

Please optimize the app configuration with "idf.py menuconfig":

* Compiler Options->Optimization Level: set to -Os (Release)
* Component Config->ESP32 Specific: set Minimum Supported ESP32 Revision to 1 (if you have a newer device...)
* Component Config->ESP32 Specific: set CPU frequency to 240 MHz
* Component Config->FreeRTOS: set Tick rate (Hz) to 1000
