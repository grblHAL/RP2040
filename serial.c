/*
  serial.c - driver code for RP2040 processor

  Part of grblHAL

  Copyright (c) 2021-2022 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#include "serial.h"
#include "driver.h"
#include "grbl/protocol.h"

#define UART_TX_PIN 0
#define UART_RX_PIN 1

#ifndef UART_PORT
#define UART_PORT uart0
#define UART ((uart_hw_t *)UART_PORT)
#define UART_IRQ UART0_IRQ
#endif

static uint16_t tx_fifo_size;
static stream_tx_buffer_t txbuf = {0};
static stream_rx_buffer_t rxbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static void uart_interrupt_handler (void);

#ifdef SERIAL2_MOD

#define UART2_TX_PIN 8
#define UART2_RX_PIN 9

#ifndef UART2_PORT
#define UART2_PORT uart1
#define UART2 ((uart_hw_t *)UART2_PORT)
#define UART2_IRQ UART1_IRQ
#endif

static stream_tx_buffer_t tx2buf = {0};
static stream_rx_buffer_t rx2buf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command2 = protocol_enqueue_realtime_command;

static void uart2_interrupt_handler (void);

#endif

static io_stream_properties_t serial[] = {
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.connected = On,
      .flags.can_set_baud = On,
      .claim = serialInit
    },
#ifdef SERIAL2_MOD
    {
      .type = StreamType_Serial,
      .instance = 1,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.connected = On,
      .flags.can_set_baud = On,
      .claim = serial2Init
    }
#endif
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    stream_register_streams(&streams);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serialGetC (void)
{
    int16_t data;
    uint_fast16_t bptr = rxbuf.tail;

    if(bptr == rxbuf.head)
        return -1; // no data available

    data = rxbuf.data[bptr];            // Get next character, increment tmp pointer
    rxbuf.tail = BUFNEXT(bptr, rxbuf);  // and update pointer

    return data;
}

static void serialTxFlush (void)
{
    txbuf.tail = txbuf.head;
}

static uint16_t serialRxCount (void)
{
    uint_fast16_t head = rxbuf.head, tail = rxbuf.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serialRxFree (void)
{
    return RX_BUFFER_SIZE - 1 - serialRxCount();
}

static void serialRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
    rxbuf.overflow = false;
}

static void serialRxCancel (void)
{
    serialRxFlush();
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

static bool serialPutC (const char c)
{
    uint_fast16_t next_head;

    if(!(UART->imsc & UART_UARTIMSC_TXIM_BITS)) {               // If the transmit interrupt is deactivated
        if(!(UART->fr & UART_UARTFR_TXFF_BITS)) {               // and if the TX FIFO is not full
            UART->dr = c;                                       // Write data in the TX FIFO
            return true;
        } else
            hw_set_bits(&UART->imsc, UART_UARTIMSC_TXIM_BITS);  // Enable transmit interrupt
    }

    // Write data in the Buffer is transmit interrupt activated or TX FIFO is                                                                
    next_head = BUFNEXT(txbuf.head, txbuf);                     // Get and update head pointer

    while(txbuf.tail == next_head) {                            // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    txbuf.data[txbuf.head] = c;                                 // Add data to buffer
    txbuf.head = next_head;                                     // and update head pointer

    return true;
}

static void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

static void serialWrite (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serialPutC(*ptr++);
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static uint16_t serialTxCount (void) {

    uint_fast16_t head = txbuf.head, tail = txbuf.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + ((UART->fr & UART_UARTFR_BUSY_BITS) ? 0 : 1);
}


static bool serialSetBaudRate (uint32_t baud_rate)
{
    uart_set_baudrate(UART_PORT, baud_rate);

    return true;
}

static bool serialDisable (bool disable)
{
    if(disable)
        hw_clear_bits(&UART->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);       
    else
        hw_set_bits(&UART->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);    
}

static bool serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.connected = On,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .disable_rx = serialDisable,
        .set_baud_rate = serialSetBaudRate,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(serial[0].flags.claimed)
        return NULL;

    serial[0].flags.claimed = On;

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    uart_init(UART_PORT, baud_rate);

    uart_set_hw_flow(UART_PORT, false, false);
    uart_set_format(UART_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_PORT, true);

    irq_set_exclusive_handler(UART_IRQ, uart_interrupt_handler);
    irq_set_enabled(UART_IRQ, true);
    
    hw_set_bits(&UART->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART,
        .pin = UART_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Primary UART"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART,
        .pin = UART_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Primary UART"
    };

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);

    return &stream;           
}

static void uart_interrupt_handler(void)
{
    uint32_t data, ctrl = UART->mis;

    if(ctrl & (UART_UARTMIS_RXMIS_BITS | UART_UARTIMSC_RTIM_BITS)) {
        while (uart_is_readable(UART_PORT)) {
            data = UART->dr & 0xFF;                                     // Read input (use only 8 bits of data)
            if(!enqueue_realtime_command((char)data)) {
                uint_fast16_t next_head = BUFNEXT(rxbuf.head, rxbuf);   // Get next head pointer
                if(next_head == rxbuf.tail)                             // If buffer full
                    rxbuf.overflow = true;                              // flag overflow
                else {
                    rxbuf.data[rxbuf.head] = (char)data;                // Add data to buffer
                    rxbuf.head = next_head;                             // and update pointer
                }
            }
        }
    }

    // Interrupt if the TX FIFO is lower or equal to the empty TX FIFO threshold
    if(ctrl & UART_UARTMIS_TXMIS_BITS)
    {
        uint_fast16_t tail = txbuf.tail;

        // As long as the TX FIFO is not full or the buffer is not empty
        while((!(UART->fr & UART_UARTFR_TXFF_BITS)) && (tail != txbuf.head)) {
            UART->dr = txbuf.data[tail];    // Put character in TX FIFO
            tail = BUFNEXT(tail, txbuf);    // and update tmp tail pointer
        }
        txbuf.tail = tail;                  //  Update tail pointer

        if(txbuf.tail == txbuf.head)	    // Disable TX interrupt when the TX buffer is empty
            hw_clear_bits(&UART->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

#ifdef SERIAL2_MOD

//
// serial2GetC - returns -1 if no data available
//
static int16_t serial2GetC (void)
{
    int16_t data;
    uint_fast16_t bptr = rx2buf.tail;

    if(bptr == rx2buf.head)
        return -1; // no data available else EOF

    data = rx2buf.data[bptr];                // Get next character
    rx2buf.tail = BUFNEXT(bptr, rx2buf);  // and update pointer

    return data;
}

static void serial2TxFlush (void)
{
    tx2buf.tail = tx2buf.head;
}

static uint16_t serial2RxCount (void)
{
    uint_fast16_t head = rx2buf.head, tail = rx2buf.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serial2RxFree (void)
{
    return RX_BUFFER_SIZE - 1 - serialRxCount();
}

static void serial2RxFlush (void)
{
    rx2buf.tail = rx2buf.head;
    rx2buf.overflow = false;
}

static void serial2RxCancel (void)
{
    serial2RxFlush();
    rx2buf.data[rx2buf.head] = ASCII_CAN;
    rx2buf.head = BUFNEXT(rx2buf.head, rx2buf);
}

static bool serial2PutC (const char c)
{
    uint_fast16_t next_head;

    if(!(UART2->imsc & UART_UARTIMSC_TXIM_BITS)) {                  // If the transmit interrupt is deactivated
        if(!(UART2->fr & UART_UARTFR_TXFF_BITS)) {                  // and if the TX FIFO is not full
            UART2->dr = c;                                          // Write data in the TX FIFO
            return true;
        } else
            hw_set_bits(&UART2->imsc, UART_UARTIMSC_TXIM_BITS);     // Enable transmit interrupt
    }

    // Write data in the Buffer is transmit interrupt activated or TX FIFO is                                                                
    next_head = BUFNEXT(tx2buf.head, tx2buf);                 // Get and update head pointer

    while(tx2buf.tail == next_head) {                             // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    tx2buf.data[tx2buf.head] = c;                             // Add data to buffer
    tx2buf.head = next_head;                                     // and update head pointer

    return true;
}

static void serial2WriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serial2PutC(c);
}

static void serial2Write (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serial2PutC(*ptr++);
}

static bool serial2SuspendInput (bool suspend)
{
    return stream_rx_suspend(&rx2buf, suspend);
}

static uint16_t serial2TxCount (void) {

    uint_fast16_t head = tx2buf.head, tail = tx2buf.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + ((UART2->fr & UART_UARTFR_BUSY_BITS) ? 0 : 1);
}

static bool serial2SetBaudRate (uint32_t baud_rate)
{
    static bool init_ok = false;

    if(!init_ok) {
        serial2Init(baud_rate);
        init_ok = true;
    }

    return true;
}

static bool serial2Disable (bool disable)
{
   if(disable)
        hw_clear_bits(&UART2->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);       
    else
        hw_set_bits(&UART->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);    
}

static bool serial2EnqueueRtCommand (char c)
{
    return enqueue_realtime_command2(c);
}

static enqueue_realtime_command_ptr serial2SetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command2;

    if(handler)
        enqueue_realtime_command2 = handler;

    return prev;
}

const io_stream_t *serial2Init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.connected = true,
        .read = serial2GetC,
        .write = serial2WriteS,
        .write_char = serial2PutC,
        .write_n = serial2Write,
        .enqueue_rt_command = serial2EnqueueRtCommand,
        .get_rx_buffer_free = serial2RxFree,
        .get_rx_buffer_count = serial2RxCount,
        .get_tx_buffer_count = serial2TxCount,
        .reset_read_buffer = serial2RxFlush,
        .cancel_read_buffer = serial2RxCancel,
        .reset_write_buffer = serial2TxFlush,
        .disable_rx = serial2Disable,
        .suspend_read = serial2SuspendInput,
        .set_baud_rate = serial2SetBaudRate,
        .set_enqueue_rt_handler = serial2SetRtHandler
    };

    if(serial[1].flags.claimed)
        return NULL;

    serial[1].flags.claimed = On;

    gpio_set_function(UART2_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART2_RX_PIN, GPIO_FUNC_UART);
    
    uart_init(UART2_PORT, baud_rate);

    uart_set_hw_flow(UART2_PORT, false, false);
    uart_set_format(UART2_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART2_PORT, true);

    irq_set_exclusive_handler(UART2_IRQ, uart2_interrupt_handler);
    irq_set_enabled(UART2_IRQ, true);
    
    hw_set_bits(&UART2->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART2,
        .pin = UART2_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Secondary UART"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART2,
        .pin = UART2_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Secondary UART"
    };

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);
    return &stream;
}

static void __not_in_flash_func(uart2_interrupt_handler)(void)
{
    uint32_t data, ctrl = UART2->mis;

    if(ctrl & (UART_UARTMIS_RXMIS_BITS | UART_UARTIMSC_RTIM_BITS)) {
        while (uart_is_readable(UART2_PORT)) {
            data = UART2->dr & 0xFF;                                    // Read input (use only 8 bits of data)
            if(!enqueue_realtime_command2((char)data)) {
                uint_fast16_t next_head = BUFNEXT(rx2buf.head, rx2buf); // Get next head pointer
                if(next_head == rx2buf.tail)                            // If buffer full
                    rx2buf.overflow = true;                             // flag overflow
                else {
                    rx2buf.data[rx2buf.head] = (char)data;              // Add data to buffer
                    rx2buf.head = next_head;                            // and update pointer
                }
            }
        }
    }

    // Interrupt if the TX FIFO is lower or equal to the empty TX FIFO threshold
    if(ctrl & UART_UARTMIS_TXMIS_BITS)
    {
        uint_fast16_t tail = tx2buf.tail;

        // As long as the TX FIFO is not full or the buffer is not empty
        while((!(UART2->fr & UART_UARTFR_TXFF_BITS)) && (tail != tx2buf.head)) {
            UART2->dr = tx2buf.data[tail];                          // Put character in TX FIFO
            tail = BUFNEXT(tail, tx2buf);                           // and update tmp tail pointer
        }
        tx2buf.tail = tail;                                         //  Update tail pointer

        if(tx2buf.tail == tx2buf.head)						        // Disable TX interrupt when the TX buffer is empty
            hw_clear_bits(&UART2->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

#endif // SERIAL2_MOD
