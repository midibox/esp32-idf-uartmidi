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

#include "uartmidi.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"
#include "esp_log.h"

#if UARTMIDI_ENABLE_CONSOLE
# include "esp_console.h"
# include "argtable3/argtable3.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

// HW and Pin assignments (can be overruled if required)
#if UARTMIDI_NUM_PORTS >= 1
# ifndef UARTMIDI_PORT0_DEV
# define UARTMIDI_PORT0_DEV UART_NUM_2
# endif
# ifndef UARTMIDI_PORT0_TXD
# define UARTMIDI_PORT0_TXD (GPIO_NUM_17)
# endif
# ifndef UARTMIDI_PORT0_RXD
# define UARTMIDI_PORT0_RXD (GPIO_NUM_16)
# endif
#endif

#if UARTMIDI_NUM_PORTS >= 2
# ifndef UARTMIDI_PORT1_DEV
# define UARTMIDI_PORT1_DEV UART_NUM_1
# endif
# ifndef UARTMIDI_PORT1_TXD
# define UARTMIDI_PORT1_TXD (GPIO_NUM_1) // same pin like UART0, since UART0 is typically used for log messages and might conflict
# endif
# ifndef UARTMIDI_PORT1_RXD
# define UARTMIDI_PORT1_RXD (GPIO_NUM_3) // same pin like UART0, since UART0 is typically used for log messages and might conflict
# endif
#endif

#if UARTMIDI_NUM_PORTS >= 3
# ifndef UARTMIDI_PORT2_DEV
# define UARTMIDI_PORT2_DEV UART_NUM_0
# endif
# ifndef UARTMIDI_PORT2_TXD
# define UARTMIDI_PORT2_TXD (GPIO_NUM_1)
# endif
# ifndef UARTMIDI_PORT2_RXD
# define UARTMIDI_PORT2_RXD (GPIO_NUM_3)
# endif
#endif


#if UARTMIDI_NUM_PORTS >= 4
# error "Driver not prepared for more than 3 ports yet!"
#endif


// Switches UART interface between MIDI and Console
// Has to be configured via Console Command "uartmidi_jumper" - overrule this define to predefine a pin w/o console
#ifndef UARTMIDI_STORAGE_NAMESPACE
#define UARTMIDI_STORAGE_NAMESPACE "MIDIbox_UART"
#endif

#ifndef UARTMIDI2_ENABLE_JUMPER_DEFAULT_PIN
# define UARTMIDI2_ENABLE_JUMPER_DEFAULT_PIN 0
#endif
static uint8_t uartmidi_enable_jumper = UARTMIDI2_ENABLE_JUMPER_DEFAULT_PIN;


// FIFO Sizes
#ifndef UARTMIDI_RX_FIFO_SIZE
#define UARTMIDI_RX_FIFO_SIZE 256
#endif

#ifndef UARTMIDI_TX_FIFO_SIZE
#define UARTMIDI_TX_FIFO_SIZE 256
#endif


typedef struct {
  int8_t     dev; // if >= 0, UART is enabled, if < 0: UART disabled (default after uartmidi_init() !)

  struct {
    uint32_t last_event_timestamp;
    uint8_t  event[3];
    uint8_t  running_status;
    uint8_t  expected_bytes;
    uint8_t  wait_bytes;
    size_t   continued_sysex_pos;
  } rx;

  struct {
    uint32_t last_event_timestamp;
    uint8_t  running_status; // running status optimization for outgoing streams not used yet...
  } tx;
} uartmidi_handle_t;

static uartmidi_handle_t uartmidi_handle[UARTMIDI_NUM_PORTS];


static void (*uartmidi_callback_midi_message_received)(uint8_t uartmidi_port, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal function to initialize the UART handle
////////////////////////////////////////////////////////////////////////////////////////////////////
static int32_t uartmidi_init_handle(uartmidi_handle_t *uart)
{
  uart->dev = -1;

  uart->rx.last_event_timestamp = 0;
  uart->rx.running_status = 0;
  uart->rx.expected_bytes = 0;
  uart->rx.wait_bytes = 0;
  uart->rx.continued_sysex_pos = 0;

  uart->tx.last_event_timestamp = 0;
  uart->tx.running_status = 0;

  return 0; // no error
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// A dummy function to prevent debug output
////////////////////////////////////////////////////////////////////////////////////////////////////
static int uartmidi_filter_log(const char *format, __VALIST vargs)
{
  return 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t uartmidi_init(void *_callback_midi_message_received)
{
  int i;
  uartmidi_handle_t *uart = &uartmidi_handle[0];
  for(i=0; i<UARTMIDI_NUM_PORTS; ++i, ++uart) {
    uartmidi_init_handle(uart);
  }

  // Finally install callback
  uartmidi_callback_midi_message_received = _callback_midi_message_received;

  esp_log_level_set(UARTMIDI_TAG, ESP_LOG_WARN); // can be changed with the "blemidi_debug on" console command

#if UARTMIDI_NUM_PORTS >= 3
  {
    uint8_t uartmidi2_enabled = 0;
    nvs_handle nvs_handle;
    esp_err_t err;

    err = nvs_open(UARTMIDI_STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if( err != ESP_OK ) {
      ESP_LOGW(UARTMIDI_TAG, "UARTMIDI Configuration not stored so far...");
    } else {
      int pin = 0;
      err = nvs_get_i32(nvs_handle, "enable_jumper", &pin);

      if( err != ESP_OK ) {
        ESP_LOGW(UARTMIDI_TAG, "Failed to restore UART MIDI Enable Jumper");
      } else {
        uartmidi_enable_jumper = pin;

        if( uartmidi_enable_jumper > 0 ) {
          // this GPIO switches UART interface between MIDI and Console
          gpio_pad_select_gpio(uartmidi_enable_jumper);
          gpio_set_direction(uartmidi_enable_jumper, GPIO_MODE_INPUT);

          // TODO: enable internal Pull-Up?
          // TODO: wait some mS?

          uartmidi2_enabled = gpio_get_level(uartmidi_enable_jumper) ? 0 : 1;
        }

        if( !uartmidi2_enabled ) {
          // Don't print if MIDI enabled
          ESP_LOGW(UARTMIDI_TAG, "Restored UART MIDI Enable Jumper at Pin: %d", uartmidi_enable_jumper);
          ESP_LOGW(UARTMIDI_TAG, "UART%d based MIDI Disabled", UARTMIDI_PORT2_DEV);
        }
      }

      if( uartmidi2_enabled ) {
        esp_log_set_vprintf(&uartmidi_filter_log);
        uartmidi_enable_port(2, 31250);
      }
    }
  }
#endif

  return 0; // no error
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// UART Port Enable (mandatory!)
////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t uartmidi_enable_port(uint8_t uartmidi_port, uint32_t baudrate)
{
  uart_config_t uart_config = {
    .baud_rate = baudrate,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //.use_ref_tick = 1,
  };

  if( uartmidi_port >= UARTMIDI_NUM_PORTS )
    return -1; // invalid port

  uartmidi_handle_t *uart = &uartmidi_handle[uartmidi_port];

  switch( uartmidi_port ) {
  case 0: {
#if UARTMIDI_NUM_PORTS >= 1
    uartmidi_init_handle(uart);
    uart->dev = UARTMIDI_PORT0_DEV;
    ESP_ERROR_CHECK(uart_param_config(UARTMIDI_PORT0_DEV, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UARTMIDI_PORT0_DEV, UARTMIDI_PORT0_TXD, UARTMIDI_PORT0_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UARTMIDI_PORT0_DEV, UARTMIDI_RX_FIFO_SIZE, UARTMIDI_TX_FIFO_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_rx_timeout(UARTMIDI_PORT0_DEV, 1)); // decrease timeout to get incoming RX data immediately
#else
    return -1; // UART not available
#endif
  } break;

  case 1: {
#if UARTMIDI_NUM_PORTS >= 2
    uartmidi_init_handle(uart);
    uart->dev = UARTMIDI_PORT1_DEV;
    ESP_ERROR_CHECK(uart_param_config(UARTMIDI_PORT1_DEV, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UARTMIDI_PORT1_DEV, UARTMIDI_PORT1_TXD, UARTMIDI_PORT1_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UARTMIDI_PORT1_DEV, UARTMIDI_RX_FIFO_SIZE, UARTMIDI_TX_FIFO_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_rx_timeout(UARTMIDI_PORT1_DEV, 1)); // decrease timeout to get incoming RX data immediately
#else
    return -1; // UART not available
#endif
  } break;

  case 2: {
#if UARTMIDI_NUM_PORTS >= 3
    uartmidi_init_handle(uart);
    uart->dev = UARTMIDI_PORT2_DEV;
    ESP_ERROR_CHECK(uart_param_config(UARTMIDI_PORT2_DEV, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UARTMIDI_PORT2_DEV, UARTMIDI_PORT2_TXD, UARTMIDI_PORT2_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UARTMIDI_PORT2_DEV, UARTMIDI_RX_FIFO_SIZE, UARTMIDI_TX_FIFO_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_rx_timeout(UARTMIDI_PORT2_DEV, 1)); // decrease timeout to get incoming RX data immediately
#else
    return -1; // UART not available
#endif
  } break;

  default: {
    return -1; // UART not available
  }
  }

  return 0; // no error
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// UART Port Disable
////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t uartmidi_disable_port(uint8_t uartmidi_port)
{
  if( uartmidi_port >= UARTMIDI_NUM_PORTS )
    return -1; // invalid port

  uartmidi_handle_t *uart = &uartmidi_handle[uartmidi_port];

  // will disable the port
  uart->dev = -1;

  switch( uartmidi_port ) {
  case 0: {
#if UARTMIDI_NUM_PORTS >= 1
    ESP_ERROR_CHECK(uart_driver_delete(UARTMIDI_PORT0_DEV));
#else
    return -1; // UART not available
#endif
  } break;

  case 1: {
#if UARTMIDI_NUM_PORTS >= 2
    ESP_ERROR_CHECK(uart_driver_delete(UARTMIDI_PORT0_DEV));
#else
    return -1; // UART not available
#endif
  } break;

  case 2: {
#if UARTMIDI_NUM_PORTS >= 3
    ESP_ERROR_CHECK(uart_driver_delete(UARTMIDI_PORT0_DEV));
#else
    return -1; // UART not available
#endif
  } break;

  default: {
    return -1; // UART not available
  }
  }

  return 0; // no error
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns != 0 if UART Port is enabled
////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t uartmidi_get_enabled(uint8_t uartmidi_port)
{
  if( uartmidi_port >= UARTMIDI_NUM_PORTS )
    return 0; // invalid port

  uartmidi_handle_t *uart = &uartmidi_handle[uartmidi_port];
  return (uart->dev >= 0) ? 1 : 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns the 1 mS based Timestamp
////////////////////////////////////////////////////////////////////////////////////////////////////
static uint64_t get_timestamp_ms()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec * 1000 + (tv.tv_usec / 1000)); // 1 mS per increment
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Send a MIDI message
////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t uartmidi_send_message(uint8_t uartmidi_port, uint8_t *stream, size_t len)
{
  if( uartmidi_port >= UARTMIDI_NUM_PORTS )
    return -1; // invalid port

  uartmidi_handle_t *uart = &uartmidi_handle[uartmidi_port];

  if( uart->dev == -1 ) {
    return -2; // UART not enabled
  }

  ESP_LOGI(UARTMIDI_TAG, "send_message uartmidi_port=%d, len=%d, stream:", uartmidi_port, len);
  esp_log_buffer_hex(UARTMIDI_TAG, stream, len);

  uart_write_bytes(uart->dev, (const char *)stream, len);
  uart->tx.last_event_timestamp = get_timestamp_ms();

  return 0; // no error
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Dummy callback for demo and debugging purposes
////////////////////////////////////////////////////////////////////////////////////////////////////
void uartmidi_receive_message_callback_for_debugging(uint8_t uartmidi_port, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
{
  ESP_LOGI(UARTMIDI_TAG, "receive_message CALLBACK uartmidi_port=%d, midi_status=0x%02x, len=%d, continued_sysex_pos=%d, remaining_message:", uartmidi_port, midi_status, len, continued_sysex_pos);
  esp_log_buffer_hex(UARTMIDI_TAG, remaining_message, len);
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// For internal usage only: receives a MIDI stream and calls the specified callback function.
// The user will specify this callback while calling uartmidi_init()
////////////////////////////////////////////////////////////////////////////////////////////////////
static int32_t uartmidi_receive_stream(uint8_t uartmidi_port, uint8_t *stream, size_t len)
{
  //! Number if expected bytes for a common MIDI event - 1
  const uint8_t midi_expected_bytes_common[8] = {
    2, // Note On
    2, // Note Off
    2, // Poly Preasure
    2, // Controller
    1, // Program Change
    1, // Channel Preasure
    2, // Pitch Bender
    0, // System Message - must be zero, so that mios32_midi_expected_bytes_system[] will be used
  };

  //! Number if expected bytes for a system MIDI event - 1
  const uint8_t midi_expected_bytes_system[16] = {
    1, // SysEx Begin (endless until SysEx End F7)
    1, // MTC Data frame
    2, // Song Position
    1, // Song Select
    0, // Reserved
    0, // Reserved
    0, // Request Tuning Calibration
    0, // SysEx End

    // Note: just only for documentation, Realtime Messages don't change the running status
    0, // MIDI Clock
    0, // MIDI Tick
    0, // MIDI Start
    0, // MIDI Continue
    0, // MIDI Stop
    0, // Reserved
    0, // Active Sense
    0, // Reset
  };

  if( uartmidi_port >= UARTMIDI_NUM_PORTS )
    return -1; // invalid port

  uartmidi_handle_t *uart = &uartmidi_handle[uartmidi_port];

  if( uart->dev == -1 ) {
    return -2; // UART not enabled
  }

  ESP_LOGI(UARTMIDI_TAG, "receive_stream uartmidi_port=%d, len=%d, stream:", uartmidi_port, len);
  esp_log_buffer_hex(UARTMIDI_TAG, stream, len);

  uint8_t message_complete = 0; // 1: common event, 2: SysEx message
  size_t pos;
  for(pos=0; pos<len; ++pos) {
    uint8_t byte = stream[pos];

    if( byte & 0x80 ) { // new MIDI status
      if( byte >= 0xf8 ) { // events >= 0xf8 don't change the running status and can just be forwarded
        // Realtime messages don't change the running status and can be sent immediately
        // They also don't touch the timeout counter!
        if( uartmidi_callback_midi_message_received ) {
          uint16_t dummy = 0; // just in case somebody always expects a pointer to at least two bytes...
          uartmidi_callback_midi_message_received(uartmidi_port, byte, (uint8_t *)&dummy, 0, 0);
        }
      } else {
        uart->rx.event[0] = byte;
        uart->rx.event[1] = 0x00;
        uart->rx.event[2] = 0x00;
        uart->rx.running_status = byte;
        uart->rx.expected_bytes = midi_expected_bytes_common[(byte >> 4) & 0x7];

        if( !uart->rx.expected_bytes ) { // System Message, take number of bytes from expected_bytes_system[] array
          uart->rx.expected_bytes = midi_expected_bytes_system[byte & 0xf];

          if( byte == 0xf0 ) {
            uart->rx.continued_sysex_pos = 0;
            // we will notify with the following bytes
          } else if( byte == 0xf7 ) {
            uart->rx.continued_sysex_pos += 1;
            message_complete = 1; // -> forward to caller
          } else if( !uart->rx.expected_bytes ) {
            // e.g. tune request (with no additional byte)
            message_complete = 1; // -> forward to caller
          }
        }

        uart->rx.wait_bytes = uart->rx.expected_bytes;
        uart->rx.last_event_timestamp = get_timestamp_ms();
      }
    } else {
      if( uart->rx.running_status == 0xf0 ) {
        message_complete = 2; // SysEx message
        uart->rx.last_event_timestamp = get_timestamp_ms();
      } else { // Common MIDI message or 0xf1 >= status >= 0xf7
        if( !uart->rx.wait_bytes ) {
          // received new MIDI event with running status
          uart->rx.wait_bytes = uart->rx.expected_bytes - 1;
        } else {
          --uart->rx.wait_bytes;
        }

        if( uart->rx.expected_bytes == 1 ) {
          uart->rx.event[1] = byte;
        } else {
          if( uart->rx.wait_bytes )
            uart->rx.event[1] = byte;
          else
            uart->rx.event[2] = byte;
        }

        if( !uart->rx.wait_bytes ) {
          uart->rx.event[0] = uart->rx.running_status;
          // midix->package.evnt1 = // already stored
          // midix->package.evnt2 = // already stored
          message_complete = 1; // -> forward to caller
        }
      }
    }

    if( message_complete == 1 ) {
      message_complete = 0;

      if( uart->rx.running_status != 0xf0 && uart->rx.running_status != 0xf7 ) {
        uart->rx.continued_sysex_pos = 0;
      }

      if( uartmidi_callback_midi_message_received ) {
        uartmidi_callback_midi_message_received(
          uartmidi_port,
          uart->rx.event[0],
          (uint8_t *)&uart->rx.event[1],
          uart->rx.expected_bytes,
          uart->rx.continued_sysex_pos);
      }
    } else if( message_complete == 2 ) { // special treatment for SysEx: forward as much as possible until 0xf7 or any other status byte
      message_complete = 0;

      int bytes_forwarded = 0;
      for(; pos<len && stream[pos] < 0x80; ++pos, ++bytes_forwarded);

      if( uartmidi_callback_midi_message_received ) {
        uartmidi_callback_midi_message_received(
          uartmidi_port,
          uart->rx.event[0],
          (uint8_t *)&stream[pos-bytes_forwarded],
          bytes_forwarded,
          uart->rx.continued_sysex_pos);
      }

      uart->rx.continued_sysex_pos += bytes_forwarded;

      if( pos < len && stream[pos] >= 0x80 ) { // special case: 0xf7 or another status byte has been received: we want the parser to process it!
        --pos;
      }
    }
  }

  return 0; // no error
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// This function should be called each mS to service incoming MIDI messages
////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t uartmidi_tick(void)
{
  uint32_t now = get_timestamp_ms();
  uint8_t buffer[UARTMIDI_RX_FIFO_SIZE];

  int i;
  uartmidi_handle_t *uart = &uartmidi_handle[0];
  for(i=0; i<UARTMIDI_NUM_PORTS; ++i, ++uart) {
    if( uart->dev >= 0 ) {
      int len = 0;
      uart_get_buffered_data_len(uart->dev, (size_t*)&len);
      if( len > 0 ) {
        if( len >= UARTMIDI_RX_FIFO_SIZE )
          len = UARTMIDI_RX_FIFO_SIZE;

        len = uart_read_bytes(uart->dev, buffer, len, 1 / portTICK_RATE_MS); // we assume no delay since len is matching with available bytes

#if 1
        uartmidi_receive_stream(i, buffer, len);
#else
        // this would be a direct loopback - an interesting option for debugging?
        uartmidi_send_message(i, buffer, len);
#endif
      }

      // timeout handling
      if( uart->rx.wait_bytes && (int32_t)(now - (uart->rx.last_event_timestamp+UARTMIDI_RX_TIMEOUT_MS)) > 0 ) {
        ESP_LOGE(UARTMIDI_TAG, "Timeout detected on uartmidi_port=%d", i);
        // TODO: we need a timeout callback?

        // at least we should clear the status
        uart->rx.running_status = 0;
        uart->rx.wait_bytes = 0;
      }
    }
  }

  return 0; // no error
}


#if UARTMIDI_ENABLE_CONSOLE
////////////////////////////////////////////////////////////////////////////////////////////////////
// Optional Console Commands
////////////////////////////////////////////////////////////////////////////////////////////////////

static struct {
  struct arg_str *on_off;
  struct arg_end *end;
} uartmidi_debug_args;

static int cmd_uartmidi_debug(int argc, char **argv)
{
  int nerrors = arg_parse(argc, argv, (void **)&uartmidi_debug_args);
  if( nerrors != 0 ) {
      arg_print_errors(stderr, uartmidi_debug_args.end, argv[0]);
      return 1;
  }

  if( strcasecmp(uartmidi_debug_args.on_off->sval[0], "on") == 0 ) {
    printf("Enable debug messages\n");
    esp_log_level_set(UARTMIDI_TAG, ESP_LOG_INFO);
  } else {
    printf("Disable debug messages - they can be re-enauartd with 'uartmidi_debug on'\n");
    esp_log_level_set(UARTMIDI_TAG, ESP_LOG_WARN);
  }

  return 0; // no error
}

static struct {
  struct arg_int *pin;
  struct arg_end *end;
} uartmidi_jumper_args;

static int cmd_uartmidi_jumper(int argc, char **argv)
{
  int nerrors = arg_parse(argc, argv, (void **)&uartmidi_jumper_args);
  if( nerrors != 0 ) {
      arg_print_errors(stderr, uartmidi_jumper_args.end, argv[0]);
      return 1;
  }

#if UARTMIDI_NUM_PORTS >= 3
  uartmidi_enable_jumper = uartmidi_jumper_args.pin->ival[0];

  if( uartmidi_enable_jumper > 0 ) {
    printf("UART%d will be enabled via jumper at pin %d with next power-on reset.\n", UARTMIDI_PORT2_DEV, uartmidi_enable_jumper);
  } else {
    printf("UART%d permanently disabled with next power-on reset (no pin assigned to jumper).\n", UARTMIDI_PORT2_DEV);
  }

  {
    nvs_handle nvs_handle;
    esp_err_t err;

    err = nvs_open(UARTMIDI_STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if( err != ESP_OK ) {
      ESP_LOGE(UARTMIDI_TAG, "UART MIDI Configuration can't be stored!");
      return -1;
    }

    err = nvs_set_i32(nvs_handle, "enable_jumper", uartmidi_enable_jumper);
    if( err != ESP_OK ) {
      ESP_LOGE(UARTMIDI_TAG, "Failed to store enable_jumper!");
      return -1;
    }
  }
#else
  printf("UART0 not available in this application.\n");
#endif

  return 0; // no error
}

void uartmidi_register_console_commands(void)
{
  {
    uartmidi_debug_args.on_off = arg_str1(NULL, NULL, "<on/off>", "Enables/Disables debug messages");
    uartmidi_debug_args.end = arg_end(20);

    const esp_console_cmd_t uartmidi_debug_cmd = {
      .command = "uartmidi_debug",
      .help = "Enables/Disables UART MIDI Debug Messages",
      .hint = NULL,
      .func = &cmd_uartmidi_debug,
      .argtable = &uartmidi_debug_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&uartmidi_debug_cmd) );
  }

  {
    uartmidi_jumper_args.pin = arg_int1(NULL, "pin", "<pin>", "GPIO Input connected to a Jumper to enable/disable UART0");
    uartmidi_jumper_args.end = arg_end(20);

    const esp_console_cmd_t uartmidi_jumper_cmd = {
      .command = "uartmidi_jumper",
      .help = "Enables/Disables UART0 via Jumper",
      .hint = NULL,
      .func = &cmd_uartmidi_jumper,
      .argtable = &uartmidi_jumper_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&uartmidi_jumper_cmd) );
  }
}

#endif

