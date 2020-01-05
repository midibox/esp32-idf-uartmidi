/*
 * UART MIDI Demo
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

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "uartmidi.h"

#define TAG "MIDIbox"

///////////////////////////////////////////////////////////////////////////////////////////////////
// This callback will be called whenever a new MIDI message has been received
////////////////////////////////////////////////////////////////////////////////////////////////////
void uartmidi_receive_message_callback(uint8_t uartmidi_port, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
{
  // enable to print out debug messages
  ESP_LOGI(TAG, "receive_message CALLBACK uartmidi_port=%d, midi_status=0x%02x, len=%d, continued_sysex_pos=%d, remaining_message:", uartmidi_port, midi_status, len, continued_sysex_pos);
  esp_log_buffer_hex(TAG, remaining_message, len);

  // loopback received message
  {
    // TODO: more comfortable packet creation via special APIs

    // Note: by intention we create new packets for each incoming message
    // this shows that running status is maintained, and that SysEx streams work as well

    if( midi_status == 0xf0 && continued_sysex_pos > 0 ) {
      uartmidi_send_message(0, remaining_message, len); // just forward
    } else {
      size_t loopback_packet_len = 1 + len; // includes MIDI status and remaining bytes
      uint8_t *loopback_packet = (uint8_t *)malloc(loopback_packet_len * sizeof(uint8_t));
      if( loopback_packet == NULL ) {
        // no memory...
      } else {
        loopback_packet[0] = midi_status;
        memcpy(&loopback_packet[1], remaining_message, len);

        uartmidi_send_message(uartmidi_port, loopback_packet, loopback_packet_len);

        free(loopback_packet);
      }
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// This task polls for new MIDI messages
////////////////////////////////////////////////////////////////////////////////////////////////////
static void task_midi(void *pvParameters)
{
  uartmidi_init(uartmidi_receive_message_callback);

  // we use uartmidi_port 0 with standard baudrate in this demo
  // this UART is connected to TX pin 17, RX pin 16
  // see also components/uartmidi/uartmidi.c
#if 1
  uartmidi_enable_port(0, 31250);
#else
  uartmidi_enable_port(1, 115200); // for "Hairless MIDI", connected to Tx Pin 1, RX Pin 3 via USB UART bridge
#endif

  while (1) {
    vTaskDelay(1 / portTICK_RATE_MS);

    uartmidi_tick();
  }
}

void app_main()
{
  xTaskCreate(task_midi, "task_midi", 4096, NULL, 8, NULL);

#if 1
  // disable this for less debug messages (avoid packet loss on loopbacks)
  // Note that they will also influence in case MIDI messages are sent via Pin1/3 (uartmidi_port 1 & 2)
  esp_log_level_set(TAG, ESP_LOG_WARN);
#endif
}
