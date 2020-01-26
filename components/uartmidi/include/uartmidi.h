/*
 * UART MIDI Driver
 *
 * See README.md for usage hints
 *
 * =============================================================================
 *
 * MIT License
 *
 * Copyright (c) 2020 Thorsten Klose (tk@midibox.org)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * =============================================================================
 */

#ifndef _UARTMIDI_H
#define _UARTMIDI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#ifndef UARTMIDI_TAG
#define UARTMIDI_TAG "UARTMIDI"
#endif  

#ifndef UARTMIDI_NUM_PORTS
#define UARTMIDI_NUM_PORTS 3
#endif

#ifndef UARTMIDI_RX_TIMEOUT_MS
#define UARTMIDI_RX_TIMEOUT_MS 2000
#endif

#ifndef UARTMIDI_ENABLE_CONSOLE
#define UARTMIDI_ENABLE_CONSOLE 1
#endif
/**
 * @brief Initializes the UART MIDI Drvier
 *
 * @param  callback_midi_message_received References the callback function which is called whenever a new MIDI message has been received.
 *         API see uartmidi_receive_packet_callback_for_debugging
 *         Specify NULL if no callback required in your application.
 *
 * @return < 0 on errors
 *
 */  
extern int32_t uartmidi_init(void *callback_midi_message_received);

/**
 * @brief Enables a UART for sending/receiving MIDI messages.
 *
 * @param  uartmidi_port  UART port
 * @param  baudrate       should be 31250 for standard MIDI baudrate, or for example 115200 for "Hairless MIDI" over USB bridge
 *
 * @return < 0 on errors
 *
 */
extern int32_t uartmidi_enable_port(uint8_t uartmidi_port, uint32_t baudrate);


/**
 * @brief Disables a UART
 *
 * @param  uartmidi_port  UART port
 *
 * @return < 0 on errors
 *
 */
extern int32_t uartmidi_disable_port(uint8_t uartmidi_port);

/**
 * @brief Returns MIDI Port Status
 *
 * @return != 0 if UART Port is enabled
 *
 */
//
////////////////////////////////////////////////////////////////////////////////////////////////////
extern int32_t uartmidi_get_enabled(uint8_t uartmidi_port);

/**
 * @brief Sends a MIDI message over UART
 *
 * @param  uartmidi_port UART port
 * @param  stream        output stream
 * @param  len           output stream length
 *
 * @return < 0 on errors
 *
 */  
extern int32_t uartmidi_send_message(uint8_t uartmidi_port, uint8_t *stream, size_t len);

/**
 * @brief A dummy callback which demonstrates the usage.
 *        It will just print out incoming MIDI messages on the terminal.
 *        You might want to implement your own for doing something more useful!

 * @param  uartmidi_port currently always 0 expected (we might support multiple ports in future)
 * @param  midi_status  the MIDI status byte (first byte of a MIDI message)
 * @param  remaining_message the remaining bytes
 * @param  len size of the remaining message
 * @param  continued_sysex_pos on ongoing SysEx stream
 *
 * @return < 0 on errors
 */  
extern void uartmidi_receive_message_callback_for_debugging(uint8_t uartmidi_port, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos);

/**
 * @brief This function should be called each mS to service incoming MIDI messages
 *
 * @return < 0 on errors
 */  
extern int32_t uartmidi_tick(void);

#if UARTMIDI_ENABLE_CONSOLE
/**
 * @brief Register Console Commands
 *
 * @return < 0 on errors
 */
extern void uartmidi_register_console_commands(void);
#endif
#ifdef __cplusplus
}
#endif

#endif /* _UARTMIDI_H */
