/*
  serial.c - driver code for RP2040 processor

  Part of grblHAL

  Copyright (c) 2021-2024 Terje Io

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
*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#include "driver.h"
#include "grbl/protocol.h"
#include "grbl/pin_bits_masks.h"

#define RX_BUFFER_HWM 800
#define RX_BUFFER_LWM 300

#ifndef UART_TX_PIN
#define UART_TX_PIN 0
#endif
#ifndef UART_RX_PIN
#define UART_RX_PIN 1
#endif

#ifndef UART_PORT
#define UART_PORT uart0
#define UART ((uart_hw_t *)UART_PORT)
#define UART_IRQ UART0_IRQ
#endif

static uint16_t tx_fifo_size;
static stream_tx_buffer_t txbuf = {0};
static stream_rx_buffer_t rxbuf = {0};
static const io_stream_t *serialInit (uint32_t baud_rate);
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;
static void uart_interrupt_handler (void);

#ifdef SERIAL1_PORT

#ifndef UART_1_TX_PIN
#define UART_1_TX_PIN 8
#endif
#ifndef UART_1_RX_PIN
#define UART_1_RX_PIN 9
#endif

#ifndef UART_1_PORT
#define UART_1_PORT uart1
#define UART_1 ((uart_hw_t *)UART_1_PORT)
#define UART_1_IRQ UART1_IRQ
#endif

static stream_tx_buffer_t tx1buf = {0};
static stream_rx_buffer_t rx1buf = {0};
static const io_stream_t *serial1Init (uint32_t baud_rate);
static enqueue_realtime_command_ptr enqueue_realtime_command2 = protocol_enqueue_realtime_command;
static void uart1_interrupt_handler (void);

#else
#define SERIAL1_PORT -1
#endif

static io_stream_properties_t serial[] = {
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
#ifdef RTS_PIN
      .flags.rts_handshake = On,
#endif
      .claim = serialInit
    },
#if SERIAL1_PORT >= 0
    {
      .type = StreamType_Serial,
      .instance = 1,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serial1Init
    }
#endif
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    static const periph_pin_t tx0 = {
        .function = Output_TX,
        .group = PinGroup_UART,
        .pin = UART_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Primary UART"
    };

    static const periph_pin_t rx0 = {
        .function = Input_RX,
        .group = PinGroup_UART,
        .pin = UART_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Primary UART"
    };

    hal.periph_port.register_pin(&rx0);
    hal.periph_port.register_pin(&tx0);

#if SERIAL1_PORT >= 0

    static const periph_pin_t tx1 = {
        .function = Output_TX,
        .group = PinGroup_UART2,
        .pin = UART_1_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Secondary UART"
    };

    static const periph_pin_t rx1 = {
        .function = Input_RX,
        .group = PinGroup_UART2,
        .pin = UART_1_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Secondary UART"
    };

    hal.periph_port.register_pin(&rx1);
    hal.periph_port.register_pin(&tx1);

#endif

    stream_register_streams(&streams);
}

static bool serialClaimPort (uint8_t instance)
{
    bool ok = false;
    uint_fast8_t idx = sizeof(serial) / sizeof(io_stream_properties_t);

    do {
        if(serial[--idx].instance == instance) {
            if((ok = serial[idx].flags.claimable && !serial[idx].flags.claimed))
                serial[idx].flags.claimed = On;
            break;
        }

    } while(idx);

    return ok;
}

// ---

static uint16_t serialRxCount (void)
{
    uint_fast16_t head = rxbuf.head, tail = rxbuf.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serialRxFree (void)
{
    return RX_BUFFER_SIZE - 1 - serialRxCount();
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

#ifdef RTS_PIN
    if(rxbuf.rts_state && serialRxCount() <= RX_BUFFER_LWM)
        DIGITAL_OUT(RTS_BIT, (rxbuf.rts_state = Off));
#endif

    return data;
}

static void serialTxFlush (void)
{
    hw_clear_bits(&UART->imsc, UART_UARTIMSC_TXIM_BITS);
    txbuf.tail = txbuf.head;
}

static void serialRxFlush (void)
{
    volatile uint32_t tmp;

    while(!(UART->fr & UART_UARTFR_RXFE_BITS))
        tmp = UART->dr & 0xFF;

    rxbuf.tail = rxbuf.head;
    rxbuf.overflow = false;

#ifdef RTS_PIN
    DIGITAL_OUT(RTS_BIT, (rxbuf.rts_state = Off));
#endif
}

static void __not_in_flash_func(serialRxCancel) (void)
{
    rxbuf.overflow = false;
    rxbuf.tail = rxbuf.head;
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
#ifdef RTS_PIN
    DIGITAL_OUT(RTS_BIT, (rxbuf.rts_state = Off));
#endif
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

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + ((UART->fr & UART_UARTFR_BUSY_BITS) ? 1 : 0);
}


static bool serialSetBaudRate (uint32_t baud_rate)
{
    uart_set_baudrate(UART_PORT, baud_rate);

    return true;
}

static bool serialSetFormat (serial_format_t format)
{
    uart_set_format(UART_PORT, format.width == Serial_8bit ? 8 : 7, format.stopbits == Serial_StopBits2 ? 2 : 1, (uart_parity_t)format.parity);

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

static const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .is_connected = stream_connected,
        .read = serialGetC,
        .write = serialWriteS,
        .write_n = serialWrite,
        .write_char = serialPutC,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .get_tx_buffer_count = serialTxCount,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .reset_write_buffer = serialTxFlush,
        .suspend_read = serialSuspendInput,
        .disable_rx = serialDisable,
        .set_baud_rate = serialSetBaudRate,
        .set_format = serialSetFormat,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(!serialClaimPort(stream.instance))
        return NULL;

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    uart_init(UART_PORT, baud_rate);

    uart_set_hw_flow(UART_PORT, false, false);
    uart_set_format(UART_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_PORT, true);

    irq_set_exclusive_handler(UART_IRQ, uart_interrupt_handler);
    irq_set_enabled(UART_IRQ, true);
    
    hw_set_bits(&UART->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);

#ifdef RTS_PIN
    DIGITAL_OUT(RTS_BIT, (rxbuf.rts_state = Off));
#endif

    return &stream;           
}

static void uart_interrupt_handler(void)
{
    uint32_t data, ctrl = UART->mis;

    if(ctrl & (UART_UARTMIS_RXMIS_BITS | UART_UARTIMSC_RTIM_BITS)) {
        while (!(UART->fr & UART_UARTFR_RXFE_BITS)) {
            data = UART->dr & 0xFF;                                     // Read input (use only 8 bits of data)
            if(!enqueue_realtime_command((char)data)) {
                uint_fast16_t next_head = BUFNEXT(rxbuf.head, rxbuf);   // Get next head pointer
                if(next_head == rxbuf.tail)                             // If buffer full
                    rxbuf.overflow = true;                              // flag overflow
                else {
                    rxbuf.data[rxbuf.head] = (char)data;                // Add data to buffer
                    rxbuf.head = next_head;                             // and update pointer
#ifdef RTS_PIN
                    if(!rxbuf.rts_state && BUFCOUNT(rxbuf.head, rxbuf.tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM)
                        DIGITAL_OUT(RTS_BIT, (rxbuf.rts_state = On));
#endif
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

#if SERIAL1_PORT >= 0

//
// serial1GetC - returns -1 if no data available
//
static int16_t serial1GetC (void)
{
    int16_t data;
    uint_fast16_t bptr = rx1buf.tail;

    if(bptr == rx1buf.head)
        return -1; // no data available else EOF

    data = rx1buf.data[bptr];                // Get next character
    rx1buf.tail = BUFNEXT(bptr, rx1buf);  // and update pointer

    return data;
}

static void serial1TxFlush (void)
{
    tx1buf.tail = tx1buf.head;
}

static uint16_t serial1RxCount (void)
{
    uint_fast16_t head = rx1buf.head, tail = rx1buf.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serial1RxFree (void)
{
    return RX_BUFFER_SIZE - 1 - serialRxCount();
}

static void serial1RxFlush (void)
{
    volatile uint32_t tmp;

    while(!(UART_1->fr & UART_UARTFR_RXFE_BITS))
        tmp = UART_1->dr & 0xFF;
 
    rx1buf.tail = rx1buf.head;
    rx1buf.overflow = false;
}

static void __not_in_flash_func(serial1RxCancel) (void)
{
    rx1buf.overflow = false;
    rx1buf.tail = rx1buf.head;
    rx1buf.data[rx1buf.head] = ASCII_CAN;
    rx1buf.head = BUFNEXT(rx1buf.head, rx1buf);
}

static bool serial1PutC (const char c)
{
    uint_fast16_t next_head;

    if(!(UART_1->imsc & UART_UARTIMSC_TXIM_BITS)) {              // If the transmit interrupt is deactivated
        if(!(UART_1->fr & UART_UARTFR_TXFF_BITS)) {              // and if the TX FIFO is not full
            UART_1->dr = c;                                      // Write data in the TX FIFO
            return true;
        } else
            hw_set_bits(&UART_1->imsc, UART_UARTIMSC_TXIM_BITS); // Enable transmit interrupt
    }

    // Write data in the Buffer is transmit interrupt activated or TX FIFO is                                                                
    next_head = BUFNEXT(tx1buf.head, tx1buf);                   // Get and update head pointer

    while(tx1buf.tail == next_head) {                           // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    tx1buf.data[tx1buf.head] = c;                               // Add data to buffer
    tx1buf.head = next_head;                                    // and update head pointer

    return true;
}

static void serial1WriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serial1PutC(c);
}

static void serial1Write (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serial1PutC(*ptr++);
}

static bool serial1SuspendInput (bool suspend)
{
    return stream_rx_suspend(&rx1buf, suspend);
}

static uint16_t serial1TxCount (void) {

    uint_fast16_t head = tx1buf.head, tail = tx1buf.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + ((UART_1->fr & UART_UARTFR_BUSY_BITS) ? 1 : 0);
}

static bool serial1SetBaudRate (uint32_t baud_rate)
{
    uart_set_baudrate(UART_1_PORT, baud_rate);

    return true;
}

static bool serial1SetFormat (serial_format_t format)
{
    uart_set_format(UART_1_PORT, format.width == Serial_8bit ? 8 : 7, format.stopbits == Serial_StopBits2 ? 2 : 1, (uart_parity_t)format.parity);

    return true;
}

static bool serial1Disable (bool disable)
{
    if(disable)
        hw_clear_bits(&UART_1->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);       
    else
        hw_set_bits(&UART_1->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);
}

static bool serial1EnqueueRtCommand (char c)
{
    return enqueue_realtime_command2(c);
}

static enqueue_realtime_command_ptr serial1SetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command2;

    if(handler)
        enqueue_realtime_command2 = handler;

    return prev;
}

static const io_stream_t *serial1Init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .instance = 1,
        .is_connected = stream_connected,
        .read = serial1GetC,
        .write = serial1WriteS,
        .write_char = serial1PutC,
        .write_n = serial1Write,
        .enqueue_rt_command = serial1EnqueueRtCommand,
        .get_rx_buffer_free = serial1RxFree,
        .get_rx_buffer_count = serial1RxCount,
        .get_tx_buffer_count = serial1TxCount,
        .reset_read_buffer = serial1RxFlush,
        .cancel_read_buffer = serial1RxCancel,
        .reset_write_buffer = serial1TxFlush,
        .disable_rx = serial1Disable,
        .suspend_read = serial1SuspendInput,
        .set_baud_rate = serial1SetBaudRate,
        .set_format = serial1SetFormat,
        .set_enqueue_rt_handler = serial1SetRtHandler
    };

    if(!serialClaimPort(stream.instance))
        return NULL;

    gpio_set_function(UART_1_TX_PIN, GPIO_FUNC_UART);
#if UART_1_RX_PIN == 27 // RP2350 - for now...
    gpio_set_function(UART_1_RX_PIN, 11);
#else
    gpio_set_function(UART_1_RX_PIN, GPIO_FUNC_UART);
#endif
    uart_init(UART_1_PORT, baud_rate);

    uart_set_hw_flow(UART_1_PORT, false, false);
    uart_set_format(UART_1_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_1_PORT, true);

    irq_set_exclusive_handler(UART_1_IRQ, uart1_interrupt_handler);
    irq_set_enabled(UART_1_IRQ, true);
    
    hw_set_bits(&UART_1->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);

    return &stream;
}

static void __not_in_flash_func(uart1_interrupt_handler)(void)
{
    uint32_t data, ctrl = UART_1->mis;

    if(ctrl & (UART_UARTMIS_RXMIS_BITS | UART_UARTIMSC_RTIM_BITS)) {
        while (!(UART_1->fr & UART_UARTFR_RXFE_BITS)) {
            data = UART_1->dr & 0xFF;                                    // Read input (use only 8 bits of data)
            if(!enqueue_realtime_command2((char)data)) {
                uint_fast16_t next_head = BUFNEXT(rx1buf.head, rx1buf); // Get next head pointer
                if(next_head == rx1buf.tail)                            // If buffer full
                    rx1buf.overflow = true;                             // flag overflow
                else {
                    rx1buf.data[rx1buf.head] = (char)data;              // Add data to buffer
                    rx1buf.head = next_head;                            // and update pointer
                }
            }
        }
    }

    // Interrupt if the TX FIFO is lower or equal to the empty TX FIFO threshold
    if(ctrl & UART_UARTMIS_TXMIS_BITS)
    {
        uint_fast16_t tail = tx1buf.tail;

        // As long as the TX FIFO is not full or the buffer is not empty
        while((!(UART_1->fr & UART_UARTFR_TXFF_BITS)) && (tail != tx1buf.head)) {
            UART_1->dr = tx1buf.data[tail];                          // Put character in TX FIFO
            tail = BUFNEXT(tail, tx1buf);                           // and update tmp tail pointer
        }
        tx1buf.tail = tail;                                         //  Update tail pointer

        if(tx1buf.tail == tx1buf.head)						        // Disable TX interrupt when the TX buffer is empty
            hw_clear_bits(&UART_1->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

#endif // SERIAL1_PORT
