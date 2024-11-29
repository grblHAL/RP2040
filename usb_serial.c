/*

  usb_serial.c - driver code for RP2040

  Part of grblHAL

  Some parts are copyright (c) 2021-2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

***

  Some parts of the code is lifted from the Pico SDK pico_stdio_usb library and are

  Copyright (c) 2020 Raspberry Pi (Trading) Ltd.

  SPDX-License-Identifier: BSD-3-Clause
  
*/

#include "tusb.h"

#include "pico/time.h"
#include "pico/binary_info.h"
#include "hardware/irq.h"

#include <string.h>

#include "usb_serial.h"
#include "driver.h"
#include "grbl/protocol.h"

//#if USB_SERIAL_CDC == 2

#define BLOCK_RX_BUFFER_SIZE 20

static stream_block_tx_buffer_t txbuf = {0};
static stream_rx_buffer_t rxbuf;
static on_execute_realtime_ptr on_execute_realtime;
static volatile enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

#ifndef PICO_STDIO_USB_STDOUT_TIMEOUT_US
#define PICO_STDIO_DEADLOCK_TIMEOUT_MS 1000
#endif

// PICO_CONFIG: PICO_STDIO_USB_STDOUT_TIMEOUT_US, Number of microseconds to be blocked trying to write USB output before assuming the host has disappeared and discarding data, default=500000, group=pico_stdio_usb
#ifndef PICO_STDIO_USB_STDOUT_TIMEOUT_US
#define PICO_STDIO_USB_STDOUT_TIMEOUT_US 500000
#endif

// todo perhaps unnecessarily high?
// PICO_CONFIG: PICO_STDIO_USB_TASK_INTERVAL_US, Period of microseconds between calling tud_task in the background, default=1000, advanced=true, group=pico_stdio_usb
#ifndef PICO_STDIO_USB_TASK_INTERVAL_US
#define PICO_STDIO_USB_TASK_INTERVAL_US 1000
#endif

// PICO_CONFIG: PICO_STDIO_USB_LOW_PRIORITY_IRQ, low priority (non hardware) IRQ number to claim for tud_task() background execution, default=31, advanced=true, group=pico_stdio_usb
#ifndef PICO_STDIO_USB_LOW_PRIORITY_IRQ
#define PICO_STDIO_USB_LOW_PRIORITY_IRQ 31
#endif

//static_assert(PICO_STDIO_USB_LOW_PRIORITY_IRQ > RTC_IRQ, ""); // note RTC_IRQ is currently the last one
static mutex_t usb_mutex;

static void execute_realtime (uint_fast16_t state);

static void low_priority_worker_irq (void)
{
    // if the mutex is already owned, then we are in user code
    // in this file which will do a tud_task itself, so we'll just do nothing
    // until the next tick; we won't starve
    if (mutex_try_enter(&usb_mutex, NULL)) {
        tud_task();
        mutex_exit(&usb_mutex);
    }
}

static int64_t timer_task (__unused alarm_id_t id, __unused void *user_data)
{
    irq_set_pending(PICO_STDIO_USB_LOW_PRIORITY_IRQ);

    return PICO_STDIO_USB_TASK_INTERVAL_US;
}

static inline bool usb_connected (void)
{
  return tud_ready();
}

static bool usb_is_connected(void)
{
    return tud_cdc_n_connected(0);
}

static void usb_out_chars (const char *buf, int length)
{
    static uint64_t last_avail_time;

    if (!mutex_try_enter_block_until(&usb_mutex, make_timeout_time_ms(PICO_STDIO_DEADLOCK_TIMEOUT_MS)))
        return;

    if (usb_connected()) {
        for (int i = 0; i < length;) {
            int n = length - i;
            int avail = tud_cdc_write_available();
            if (n > avail)
                n = avail;
            if (n) {
                int n2 = tud_cdc_write(buf + i, n);
                tud_task();
                tud_cdc_write_flush();
                i += n2;
                last_avail_time = time_us_64();
            } else {
                tud_task();
                tud_cdc_write_flush();
                if (!usb_connected() ||
                    (!tud_cdc_write_available() && time_us_64() > last_avail_time + PICO_STDIO_USB_STDOUT_TIMEOUT_US)) {
                    break;
                }
            }
        }
    } else // reset our timeout
        last_avail_time = 0;

    mutex_exit(&usb_mutex);
}

static int32_t usb_in_chars (char *buf, uint32_t length)
{
    uint32_t count = 0;

    if (usb_connected() && tud_cdc_available()) {
 
        if (!mutex_try_enter_block_until(&usb_mutex, make_timeout_time_ms(PICO_STDIO_DEADLOCK_TIMEOUT_MS)))
            return PICO_ERROR_NO_DATA; // would deadlock otherwise

        if (usb_connected() && tud_cdc_available())
            count = tud_cdc_read(buf, length);
        else // because our mutex use may starve out the background task, run tud_task here (we own the mutex)
            tud_task();

        mutex_exit(&usb_mutex);
    }

    return count ? count : PICO_ERROR_NO_DATA;
}

//
// Returns number of characters in USB input buffer
//
static uint16_t usb_serialRxCount (void)
{
    uint_fast16_t tail = rxbuf.tail, head = rxbuf.head;

    return (uint16_t)BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of free characters in USB input buffer
//
static uint16_t usb_serialRxFree (void)
{
    uint_fast16_t tail = rxbuf.tail, head = rxbuf.head;
 
    return (uint16_t)((RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE));
}

//
// Flushes the USB input buffer (including the USB buffer)
//
static void usb_serialRxFlush (void)
{
  //  usb_serial_flush_input();
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the USB input buffer
//
static void usb_serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = CMD_RESET;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes a character to the USB output stream
//
static bool usb_serialPutC (const char c)
{
    static uint8_t buf[1];

    *buf = c;
    usb_out_chars(buf, 1);

    return true;
}

bool _usb_write (void)
{
    size_t txfree, length;

    txbuf.s = txbuf.data;

    while(txbuf.length) {

        if((txfree = tud_cdc_write_available()) > 10) {

            length = txfree < txbuf.length ? txfree : txbuf.length;

            usb_out_chars(txbuf.s, length); //

            txbuf.length -= length;
            txbuf.s += length;
        }

        if(txbuf.length && !hal.stream_blocking_callback()) {
            txbuf.length = 0;
            txbuf.s = txbuf.data;
            return false;
        }
    }

    txbuf.s = txbuf.data;

    return true;
}

//
// Writes a number of characters from string to the USB output stream, blocks if buffer full
//
static void usb_serialWrite (const char *s, uint16_t length)
{
    // Empty buffer first...
    if(txbuf.length && !_usb_write())
        return;

    usb_out_chars(s, length);
}

//
// Writes a null terminated string to the USB output stream, blocks if buffer full
//
static void usb_serialWriteS (const char *s)
{
    if(*s == '\0')
        return;

    size_t length = strlen(s);

    if((length + txbuf.length) < BLOCK_TX_BUFFER_SIZE) {

        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;

        if(s[length - 1] == ASCII_LF || txbuf.length > txbuf.max_length) {
            if(!_usb_write())
                return;
        }
    } else
        usb_serialWrite(s, length);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t usb_serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;

    if(tail == rxbuf.head)
        return -1; // no data available

    char data = rxbuf.data[tail];       // Get next character, increment tmp pointer
    rxbuf.tail = BUFNEXT(tail, rxbuf); // and update pointer

    return (int16_t)data;
}

static bool usb_serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool usbEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr usb_serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *usb_serialInit (void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.is_usb = On,
        .is_connected = usb_is_connected,
        .read = usb_serialGetC,
        .write = usb_serialWriteS,
        .write_n = usb_serialWrite,
        .write_char = usb_serialPutC,
        .enqueue_rt_command = usbEnqueueRtCommand,
        .get_rx_buffer_free = usb_serialRxFree,
        .get_rx_buffer_count = usb_serialRxCount,
        .reset_read_buffer = usb_serialRxFlush,
        .cancel_read_buffer = usb_serialRxCancel,
        .suspend_read = usb_serialSuspendInput,
        .set_enqueue_rt_handler = usb_serialSetRtHandler
    };

    // initialize TinyUSB
    tusb_init();

    irq_set_exclusive_handler(PICO_STDIO_USB_LOW_PRIORITY_IRQ, low_priority_worker_irq);
    irq_set_enabled(PICO_STDIO_USB_LOW_PRIORITY_IRQ, true);

    mutex_init(&usb_mutex);
    bool rc = add_alarm_in_us(PICO_STDIO_USB_TASK_INTERVAL_US, timer_task, NULL, true);

    txbuf.s = txbuf.data;
    txbuf.max_length = CFG_TUD_CDC_TX_BUFSIZE;
    txbuf.max_length = (txbuf.max_length > BLOCK_TX_BUFFER_SIZE ? BLOCK_TX_BUFFER_SIZE : txbuf.max_length) - 20;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = execute_realtime;

    return &stream;
}

//
// This function get called from the foreground process,
// used here to get characters off the USB serial input stream and buffer
// them for processing by grbl. Real time command characters are stripped out
// and submitted for realtime processing.
//
static void execute_realtime (uint_fast16_t state)
{
    static volatile bool lock = false;
    static char tmpbuf[BLOCK_RX_BUFFER_SIZE];

    on_execute_realtime(state);

    if(lock)
        return;

    char c, *dp;
    int32_t avail, free;
 
    lock = true;
 
    if(usb_connected() && (avail = (int32_t)tud_cdc_available())) {

        dp = tmpbuf;
        free = (int32_t)usb_serialRxFree();
        free = free > BLOCK_RX_BUFFER_SIZE ? BLOCK_RX_BUFFER_SIZE : free;
        avail = usb_in_chars(tmpbuf, (uint32_t)(avail > free ? free : avail));

        if(avail > 0) while(avail--) {
            c = *dp++;
            if(!enqueue_realtime_command(c)) {
                uint_fast16_t next_head = BUFNEXT(rxbuf.head, rxbuf);   // Get next head pointer
                if(next_head == rxbuf.tail)                             // If buffer full
                    rxbuf.overflow = On;                                // flag overflow,
                else {
                    rxbuf.data[rxbuf.head] = c;                         // else add character data to buffer
                    rxbuf.head = next_head;                             // and update pointer
                }
            }
        }
    }

    lock = false;
}
