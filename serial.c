/*
  serial.c - driver code for RP2040 processor

  Part of grblHAL

  Copyright (c) 2021-2026 Terje Io

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

#if defined(SERIAL1_PORT_PIO) || defined(SERIAL2_PORT)

// NOTE: the PIO UART implementation uses two state machines and
//       may cause grblHAL initialisation to fail for some configurations.

#include "pico/time.h"
#include "hardware/pio.h"
#include "driverPIO.pio.h"

typedef struct {
    PIO pio_rx;
    uint sm_rx;
    bool rx_enabled;
    PIO pio_tx;
    uint offset_tx;
    uint sm_tx;
    uint64_t tx_deadline;
   uint32_t char_time_us;
} pio_uart_t;

#endif

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
static enqueue_realtime_command_ptr enqueue_realtime_command;
static void uart_interrupt_handler (void);

#ifdef SERIAL1_PORT

#ifndef UART_1_TX_PIN
#define UART_1_TX_PIN 8
#endif
#ifndef UART_1_RX_PIN
#define UART_1_RX_PIN 9
#endif

static stream_tx_buffer_t tx1buf = {0};
static stream_rx_buffer_t rx1buf = {0};
static const io_stream_t *serial1Init (uint32_t baud_rate);
static enqueue_realtime_command_ptr enqueue_realtime_command2;
static void uart1_interrupt_handler (void);

#ifdef SERIAL1_PORT_PIO
static pio_uart_t pio_uart1 = {0};
#else
#ifndef UART_1_PORT
#define UART_1_PORT uart1
#define UART_1 ((uart_hw_t *)UART_1_PORT)
#define UART_1_IRQ UART1_IRQ
#endif
#endif

#else
#define SERIAL1_PORT -1
#endif

#ifdef SERIAL2_PORT

#ifndef UART_2_RX_PIN
#define UART_2_RX_PIN 9
#endif

static pio_uart_t ppio_uart2 = {0};
static stream_rx_buffer_t rx2buf = {0};
#ifdef UART_2_TX_PIN
static stream_tx_buffer_t tx2buf = {0};
#endif
static const io_stream_t *serial2Init (uint32_t baud_rate);
static enqueue_realtime_command_ptr enqueue_realtime_command3;
static void uart2_interrupt_handler (void);

#else
#define SERIAL2_PORT -1
#endif

static bool uart_release (uint8_t instance);
static const io_stream_status_t *get_uart_status (uint8_t instance);

static io_stream_status_t stream_status[] = {
    {
        .baud_rate = 115200,
        .format = {
            .width = Serial_8bit,
            .stopbits = Serial_StopBits1,
            .parity = Serial_ParityNone,
        }
    },
#if SERIAL1_PORT >= 0
    {
        .baud_rate = 115200,
        .format = {
            .width = Serial_8bit,
            .stopbits = Serial_StopBits1,
            .parity = Serial_ParityNone,
        }
    },
#endif
#if SERIAL2_PORT >= 0
    {
        .baud_rate = 115200,
        .format = {
            .width = Serial_8bit,
            .stopbits = Serial_StopBits1,
            .parity = Serial_ParityNone,
        }
    }
#endif
};

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
      .claim = serialInit,
      .release = uart_release,
      .get_status = get_uart_status
    },
#if SERIAL1_PORT >= 0
    {
      .type = StreamType_Serial,
      .instance = 1,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serial1Init,
      .release = uart_release,
      .get_status = get_uart_status
    },
#endif
#if SERIAL2_PORT >= 0
    {
      .type = StreamType_Serial,
      .instance = 2,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
#ifdef UART_2_TX_PIN
      .flags.modbus_ready = On,
#else
      .flags.rx_only = On,
#endif
      .claim = serial2Init,
      .release = uart_release,
      .get_status = get_uart_status
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
        .mode = { .mask = PINMODE_OUTPUT }
    };

    static const periph_pin_t rx0 = {
        .function = Input_RX,
        .group = PinGroup_UART,
        .pin = UART_RX_PIN,
        .mode = { .mask = PINMODE_NONE }
    };

    hal.periph_port.register_pin(&rx0);
    hal.periph_port.register_pin(&tx0);

#if SERIAL1_PORT >= 0

    static const periph_pin_t tx1 = {
        .function = Output_TX,
        .group = PinGroup_UART2,
        .pin = UART_1_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT }
    };

    static const periph_pin_t rx1 = {
        .function = Input_RX,
        .group = PinGroup_UART2,
        .pin = UART_1_RX_PIN,
        .mode = { .mask = PINMODE_NONE }
    };

    hal.periph_port.register_pin(&rx1);
    hal.periph_port.register_pin(&tx1);

#endif

#if SERIAL2_PORT > 0

    static const periph_pin_t rx2 = {
        .function = Input_RX,
        .group = PinGroup_UART3,
        .pin = UART_2_RX_PIN,
        .mode = { .mask = PINMODE_NONE }
    };

    hal.periph_port.register_pin(&rx2);

#ifdef UART_2_TX_PIN

    static const periph_pin_t tx2 = {
        .function = Output_TX,
        .group = PinGroup_UART3,
        .pin = UART_2_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT }
    };

    hal.periph_port.register_pin(&tx2);

#endif
#endif

    stream_register_streams(&streams);
}

static io_stream_properties_t *get_port (uint8_t instance)
{
    uint_fast8_t i = sizeof(serial) / sizeof(io_stream_properties_t); 

    do {
        if(serial[--i].instance == instance)
            return &serial[i];
    } while(i);

    return NULL;
}

static const io_stream_status_t *get_uart_status (uint8_t instance)
{
    io_stream_properties_t *port = get_port(instance);

    if(port) 
        stream_status[instance].flags = port->flags;

    return port ? &stream_status[port - serial] : NULL;
}

static bool uart_release (uint8_t instance)
{
    bool ok;
    io_stream_properties_t *port = get_port(instance);

    if((ok = port && port->flags.claimed))
        port->flags.claimed = Off;

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
// serialGetC - returns SERIAL_NO_DATA (-1) if no data available
//
static int32_t serialGetC (void)
{
    uint_fast16_t bptr = rxbuf.tail;

    if(bptr == rxbuf.head)
        return SERIAL_NO_DATA; // no data available

    int32_t data = (int32_t)rxbuf.data[bptr];   // Get next character, increment tmp pointer
    rxbuf.tail = BUFNEXT(bptr, rxbuf);          // and update pointer

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

static bool serialPutC (const uint8_t c)
{
    uint_fast16_t next_head;

    if(!(UART->imsc & UART_UARTIMSC_TXIM_BITS)) {               // If the transmit interrupt is deactivated
        if(!(UART->fr & UART_UARTFR_TXFF_BITS)) {               // and if the TX FIFO is not full
            UART->dr = c;                                       // Write data in the TX FIFO
            return true;
        } else
            hw_set_bits(&UART->imsc, UART_UARTIMSC_TXIM_BITS);  // Enable transmit interrupt
    }

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
    uint8_t c, *ptr = (uint8_t *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

static void serialWrite (const uint8_t *s, uint16_t length)
{
    uint8_t *ptr = (uint8_t *)s;

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
    stream_status[0].baud_rate = baud_rate;

    uart_set_baudrate(UART_PORT, baud_rate);

    return true;
}

static bool serialSetFormat (serial_format_t format)
{
    stream_status[0].format = format;

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

static bool serialEnqueueRtCommand (uint8_t c)
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

    io_stream_properties_t *port = get_port(0);
    if(!port->flags.claimable || port->flags.claimed)
        return NULL;

    port->flags.claimed = On;

    if(!port->flags.init_ok) {

#if RP_MCU == 2350 && (UART_TX_PIN % 4) == 2
        gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART_AUX);
#else
        gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
#endif
#if RP_MCU == 2350 && (UART_RX_PIN % 4) == 3
        gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART_AUX);
#else
        gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
#endif
        uart_init(UART_PORT, baud_rate);

        uart_set_hw_flow(UART_PORT, false, false);
        uart_set_format(UART_PORT, 8, 1, UART_PARITY_NONE);
        uart_set_fifo_enabled(UART_PORT, true);

        serialRxFlush();
        irq_set_exclusive_handler(UART_IRQ, uart_interrupt_handler);
        irq_set_enabled(UART_IRQ, true);

        port->flags.init_ok = On;
    }

    stream_set_defaults(&stream, baud_rate);

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
            if(!enqueue_realtime_command((uint8_t)data)) {
                uint_fast16_t next_head = BUFNEXT(rxbuf.head, rxbuf);   // Get next head pointer
                if(next_head == rxbuf.tail)                             // If buffer full
                    rxbuf.overflow = true;                              // flag overflow
                else {
                    rxbuf.data[rxbuf.head] = (uint8_t)data;             // Add data to buffer
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

        if(txbuf.tail == txbuf.head)        // Disable TX interrupt when the TX buffer is empty
            hw_clear_bits(&UART->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

#if SERIAL1_PORT >= 0

static bool serial1PutC (const uint8_t c);

//
// serial1GetC - returns SERIAL_NO_DATA (-1) if no data available
//
static int32_t serial1GetC (void)
{
    uint_fast16_t bptr = rx1buf.tail;

    if(bptr == rx1buf.head)
        return SERIAL_NO_DATA; // no data available

    int32_t data = (int32_t)rx1buf.data[bptr];    // Get next character
    rx1buf.tail = BUFNEXT(bptr, rx1buf);          // and update pointer

    return data;
}

static uint16_t serial1RxCount (void)
{
    uint_fast16_t head = rx1buf.head, tail = rx1buf.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serial1RxFree (void)
{
    return RX_BUFFER_SIZE - 1 - serial1RxCount();
}

static void serial1WriteS (const char *data)
{
    uint8_t c, *ptr = (uint8_t *)data;

    while((c = *ptr++) != '\0')
        serial1PutC(c);
}

static void serial1Write (const uint8_t *s, uint16_t length)
{
    uint8_t *ptr = (uint8_t *)s;

    while(length--)
        serial1PutC(*ptr++);
}

static bool serial1SuspendInput (bool suspend)
{
    return stream_rx_suspend(&rx1buf, suspend);
}

static bool serial1EnqueueRtCommand (uint8_t c)
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

static void __not_in_flash_func(serial1RxCancel) (void)
{
    rx1buf.overflow = false;
    rx1buf.tail = rx1buf.head;
    rx1buf.data[rx1buf.head] = ASCII_CAN;
    rx1buf.head = BUFNEXT(rx1buf.head, rx1buf);
}

#ifdef SERIAL1_PORT_PIO // PIO version of SERIAL1_PORT peripheral interface

static void serial1TxFlush (void)
{
    pio_set_irqn_source_enabled(pio_uart1.pio_tx, 1, pio_get_tx_fifo_not_full_interrupt_source(pio_uart1.sm_tx), false);
    pio_sm_set_enabled(pio_uart1.pio_tx, pio_uart1.sm_tx, false);
    pio_sm_clear_fifos(pio_uart1.pio_tx, pio_uart1.sm_tx);
    pio_sm_restart(pio_uart1.pio_tx, pio_uart1.sm_tx);
    pio_sm_exec(pio_uart1.pio_tx, pio_uart1.sm_tx, pio_encode_jmp(pio_uart1.offset_tx));
    pio_sm_set_enabled(pio_uart1.pio_tx, pio_uart1.sm_tx, true);
    pio_uart1.tx_deadline = 0;

    tx1buf.tail = tx1buf.head;
}

static void serial1RxFlush (void)
{
    while(!pio_sm_is_rx_fifo_empty(pio_uart1.pio_rx, pio_uart1.sm_rx))
        (void)*((io_rw_8 *)&pio_uart1.pio_rx->rxf[pio_uart1.sm_rx] + 3);

    rx1buf.tail = rx1buf.head;
    rx1buf.overflow = false;
}

static bool serial1PutC (const uint8_t c)
{
    uint_fast16_t next_head;

    if(pio_sm_is_tx_fifo_empty(pio_uart1.pio_tx, pio_uart1.sm_tx) && tx1buf.tail == tx1buf.head) {
        pio_sm_put(pio_uart1.pio_tx, pio_uart1.sm_tx, (uint32_t)c);
        pio_uart1.tx_deadline = max(pio_uart1.tx_deadline, time_us_64()) + pio_uart1.char_time_us;
        return true;
    }

    next_head = BUFNEXT(tx1buf.head, tx1buf);                   // Get and update head pointer

    while(tx1buf.tail == next_head) {                           // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    tx1buf.data[tx1buf.head] = c;                               // Add data to buffer
    tx1buf.head = next_head;                                    // and update head pointer

    pio_set_irqn_source_enabled(pio_uart1.pio_tx, 1, pio_get_tx_fifo_not_full_interrupt_source(pio_uart1.sm_tx), true);

    return true;
}

static uint16_t serial1TxCount (void) {

    uint_fast16_t head = tx1buf.head, tail = tx1buf.tail;

    uint16_t count = BUFCOUNT(head, tail, TX_BUFFER_SIZE);

    count += (uint16_t)pio_sm_get_tx_fifo_level(pio_uart1.pio_tx, pio_uart1.sm_tx);
    if(pio_uart1.tx_deadline > time_us_64())
        count++;

    return count;
}

static bool serial1SetBaudRate (uint32_t baud_rate)
{
    stream_status[1].baud_rate = baud_rate;

    pio_uart1.char_time_us = (10 * 1000000UL + (baud_rate - 1)) / baud_rate;
    uart_tx_program_set_baud(pio_uart1.pio_tx, pio_uart1.sm_tx, baud_rate);
    uart_rx_program_set_baud(pio_uart1.pio_rx, pio_uart1.sm_rx, baud_rate);

    return true;
}

static bool serial1SetFormat (serial_format_t format)
{
    (void)format;
    return false;
}

static bool serial1Disable (bool disable)
{
    pio_uart1.rx_enabled = !disable;

    pio_set_irqn_source_enabled(pio_uart1.pio_rx, 1, pio_get_rx_fifo_not_empty_interrupt_source(pio_uart1.sm_rx), !disable);

    return true;
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
        .set_format = NULL, // only 8N1 is supported
        .set_enqueue_rt_handler = serial1SetRtHandler
    };

    io_stream_properties_t *port = get_port(1);
    if(!port->flags.claimable || port->flags.claimed)
        return NULL;

    if(!port->flags.init_ok) {

        bool ok;
        int irq_tx, irq_rx, offset_rx;

        if((ok = pio_claim_free_sm_and_add_program_for_gpio_range(&uart_tx_program, &pio_uart1.pio_tx, &pio_uart1.sm_tx, &pio_uart1.offset_tx, UART_1_TX_PIN, 1, true))) {
            irq_tx = pio_get_irq_num(pio_uart1.pio_tx, 1);
            uart_tx_program_init(pio_uart1.pio_tx, pio_uart1.sm_tx, pio_uart1.offset_tx, UART_1_TX_PIN, baud_rate);
        }

        if((ok = ok && pio_claim_free_sm_and_add_program_for_gpio_range(&uart_rx_program, &pio_uart1.pio_rx, &pio_uart1.sm_rx, &offset_rx, UART_1_RX_PIN, 1, true))) {
            irq_rx = pio_get_irq_num(pio_uart1.pio_rx, 1);
            uart_rx_program_init(pio_uart1.pio_rx, pio_uart1.sm_rx, offset_rx, UART_1_RX_PIN, baud_rate);
        } else
            return NULL;

        serial1RxFlush();
        irq_set_exclusive_handler(irq_rx, uart1_interrupt_handler);
        irq_set_enabled(irq_rx, true);
        if(irq_tx != irq_rx) {
            irq_set_exclusive_handler(irq_tx, uart1_interrupt_handler);
            irq_set_enabled(irq_tx, true);
        }
        pio_set_irqn_source_enabled(pio_uart1.pio_rx, 1, pio_get_rx_fifo_not_empty_interrupt_source(pio_uart1.sm_rx), true);

        port->flags.init_ok = On;
    }

    port->flags.claimed = On;
    pio_uart1.rx_enabled = true;

    stream_set_defaults(&stream, baud_rate);

    return &stream;
}

static void __not_in_flash_func(uart1_interrupt_handler)(void)
{
    if(pio_uart1.rx_enabled) {
        io_rw_8 *rxfifo_shift = (io_rw_8 *)&pio_uart1.pio_rx->rxf[pio_uart1.sm_rx] + 3;

        while(!pio_sm_is_rx_fifo_empty(pio_uart1.pio_rx, pio_uart1.sm_rx)) {
            uint8_t data = *rxfifo_shift;

            if(!enqueue_realtime_command2(data)) {
                uint_fast16_t next_head = BUFNEXT(rx1buf.head, rx1buf);

                if(next_head == rx1buf.tail)
                    rx1buf.overflow = true;
                else {
                    rx1buf.data[rx1buf.head] = data;
                    rx1buf.head = next_head;
                }
            }
        }
    }

    uint_fast16_t tail = tx1buf.tail;

    while(!pio_sm_is_tx_fifo_full(pio_uart1.pio_tx, pio_uart1.sm_tx) && tail != tx1buf.head) {
        pio_sm_put(pio_uart1.pio_tx, pio_uart1.sm_tx, tx1buf.data[tail]);
        pio_uart1.tx_deadline = max(pio_uart1.tx_deadline, time_us_64()) + pio_uart1.char_time_us;
        tail = BUFNEXT(tail, tx1buf);
    }

    tx1buf.tail = tail;

    if(tx1buf.tail == tx1buf.head)
        pio_set_irqn_source_enabled(pio_uart1.pio_tx, 1, pio_get_tx_fifo_not_full_interrupt_source(pio_uart1.sm_tx), false);
}

#else // UART version of SERIAL1_PORT peripheral interface

static void serial1TxFlush (void)
{
    tx1buf.tail = tx1buf.head;
}

static void serial1RxFlush (void)
{
    volatile uint32_t tmp;

    while(!(UART_1->fr & UART_UARTFR_RXFE_BITS))
        tmp = UART_1->dr & 0xFF;
 
    rx1buf.tail = rx1buf.head;
    rx1buf.overflow = false;
}

static bool serial1PutC (const uint8_t c)
{
    uint_fast16_t next_head;

    if(!(UART_1->imsc & UART_UARTIMSC_TXIM_BITS)) {              // If the transmit interrupt is deactivated
        if(!(UART_1->fr & UART_UARTFR_TXFF_BITS)) {              // and if the TX FIFO is not full
            UART_1->dr = c;                                      // Write data in the TX FIFO
            return true;
        } else
            hw_set_bits(&UART_1->imsc, UART_UARTIMSC_TXIM_BITS); // Enable transmit interrupt
    }

    next_head = BUFNEXT(tx1buf.head, tx1buf);                   // Get and update head pointer

    while(tx1buf.tail == next_head) {                           // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    tx1buf.data[tx1buf.head] = c;                               // Add data to buffer
    tx1buf.head = next_head;                                    // and update head pointer

    return true;
}

static uint16_t serial1TxCount (void) {

    uint_fast16_t head = tx1buf.head, tail = tx1buf.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + ((UART_1->fr & UART_UARTFR_BUSY_BITS) ? 1 : 0);
}

static bool serial1SetBaudRate (uint32_t baud_rate)
{
    stream_status[1].baud_rate = baud_rate;

    uart_set_baudrate(UART_1_PORT, baud_rate);

    return true;
}

static bool serial1SetFormat (serial_format_t format)
{
    stream_status[1].format = format;

    uart_set_format(UART_1_PORT, format.width == Serial_8bit ? 8 : 7, format.stopbits == Serial_StopBits2 ? 2 : 1, (uart_parity_t)format.parity);

    return true;
}

static bool serial1Disable (bool disable)
{
    if(disable)
        hw_clear_bits(&UART_1->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);       
    else
        hw_set_bits(&UART_1->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);

    return true;
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

    io_stream_properties_t *port = get_port(1);
    if(!port->flags.claimable || port->flags.claimed)
        return NULL;

    port->flags.claimed = On;

    if(!port->flags.init_ok) {

#if RP_MCU == 2350 && (UART_1_TX_PIN % 4) == 2
        gpio_set_function(UART_1_TX_PIN, GPIO_FUNC_UART_AUX);
#else
        gpio_set_function(UART_1_TX_PIN, GPIO_FUNC_UART);
#endif
#if RP_MCU == 2350 && (UART_1_RX_PIN % 4) == 3
        gpio_set_function(UART_1_RX_PIN, GPIO_FUNC_UART_AUX);
#else
        gpio_set_function(UART_1_RX_PIN, GPIO_FUNC_UART);
#endif
        uart_init(UART_1_PORT, baud_rate);

        uart_set_hw_flow(UART_1_PORT, false, false);
        uart_set_format(UART_1_PORT, 8, 1, UART_PARITY_NONE);
        uart_set_fifo_enabled(UART_1_PORT, true);

        serial1RxFlush();
        irq_set_exclusive_handler(UART_1_IRQ, uart1_interrupt_handler);
        irq_set_enabled(UART_1_IRQ, true);

        port->flags.init_ok = On;
    }

    stream_set_defaults(&stream, baud_rate);

    return &stream;
}

static void __not_in_flash_func(uart1_interrupt_handler)(void)
{
    uint32_t data, ctrl = UART_1->mis;

    if(ctrl & (UART_UARTMIS_RXMIS_BITS | UART_UARTIMSC_RTIM_BITS)) {
        while (!(UART_1->fr & UART_UARTFR_RXFE_BITS)) {
            data = UART_1->dr & 0xFF;                                   // Read input (use only 8 bits of data)
            if(!enqueue_realtime_command2((uint8_t)data)) {
                uint_fast16_t next_head = BUFNEXT(rx1buf.head, rx1buf); // Get next head pointer
                if(next_head == rx1buf.tail)                            // If buffer full
                    rx1buf.overflow = true;                             // flag overflow
                else {
                    rx1buf.data[rx1buf.head] = (uint8_t)data;           // Add data to buffer
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
            UART_1->dr = tx1buf.data[tail];                         // Put character in TX FIFO
            tail = BUFNEXT(tail, tx1buf);                           // and update tmp tail pointer
        }
        tx1buf.tail = tail;                                         //  Update tail pointer

        if(tx1buf.tail == tx1buf.head)                              // Disable TX interrupt when the TX buffer is empty
            hw_clear_bits(&UART_1->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

#endif // UART version of SERIAL1_PORT

#endif // SERIAL1_PORT

// PIO based UART

#if SERIAL2_PORT >= 0

//
// serial2GetC - returns SERIAL_NO_DATA (-1) if no data available
//
static int32_t serial2GetC (void)
{
    uint_fast16_t bptr = rx2buf.tail;

    if(bptr == rx2buf.head)
        return SERIAL_NO_DATA; // no data available

    int32_t data = (int32_t)rx2buf.data[bptr];    // Get next character
    rx2buf.tail = BUFNEXT(bptr, rx2buf);          // and update pointer

    return data;
}

static uint16_t serial2RxCount (void)
{
    uint_fast16_t head = rx2buf.head, tail = rx2buf.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serial2RxFree (void)
{
    return RX_BUFFER_SIZE - 1 - serial2RxCount();
}

#ifdef UART_2_TX_PIN

static bool serial2PutC (const uint8_t c)
{
    uint_fast16_t next_head;

    if(pio_sm_is_tx_fifo_empty(ppio_uart2.pio_tx, ppio_uart2.sm_tx) && tx2buf.tail == tx2buf.head) {
        pio_sm_put(ppio_uart2.pio_tx, ppio_uart2.sm_tx, (uint32_t)c);
        ppio_uart2.tx_deadline = max(ppio_uart2.tx_deadline, time_us_64()) + ppio_uart2.char_time_us;
        return true;
    }

    next_head = BUFNEXT(tx2buf.head, tx2buf);                   // Get and update head pointer

    while(tx2buf.tail == next_head) {                           // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    tx2buf.data[tx2buf.head] = c;                               // Add data to buffer
    tx2buf.head = next_head;                                    // and update head pointer

    pio_set_irqn_source_enabled(ppio_uart2.pio_tx, 1, pio_get_tx_fifo_not_full_interrupt_source(ppio_uart2.sm_tx), true);

    return true;
}

static void serial2WriteS (const char *data)
{
    uint8_t c, *ptr = (uint8_t *)data;

    while((c = *ptr++) != '\0')
        serial2PutC(c);
}

static void serial2Write (const uint8_t *s, uint16_t length)
{
    uint8_t *ptr = (uint8_t *)s;

    while(length--)
        serial2PutC(*ptr++);
}

static uint16_t serial2TxCount (void) {

    uint_fast16_t head = tx2buf.head, tail = tx2buf.tail;

    uint16_t count = BUFCOUNT(head, tail, TX_BUFFER_SIZE);

    count += (uint16_t)pio_sm_get_tx_fifo_level(ppio_uart2.pio_tx, ppio_uart2.sm_tx);
    if(ppio_uart2.tx_deadline > time_us_64())
        count++;

    return count;
}

static void serial2TxFlush (void)
{

    pio_set_irqn_source_enabled(ppio_uart2.pio_tx, 1, pio_get_tx_fifo_not_full_interrupt_source(ppio_uart2.sm_tx), false);
    pio_sm_set_enabled(ppio_uart2.pio_tx, ppio_uart2.sm_tx, false);
    pio_sm_clear_fifos(ppio_uart2.pio_tx, ppio_uart2.sm_tx);
    pio_sm_restart(ppio_uart2.pio_tx, ppio_uart2.sm_tx);
    pio_sm_exec(ppio_uart2.pio_tx, ppio_uart2.sm_tx, pio_encode_jmp(ppio_uart2.offset_tx));
    pio_sm_set_enabled(ppio_uart2.pio_tx, ppio_uart2.sm_tx, true);
    ppio_uart2.tx_deadline = 0;

    tx2buf.tail = tx2buf.head;
}

#endif

static bool serial2SuspendInput (bool suspend)
{
    return stream_rx_suspend(&rx2buf, suspend);
}

static bool serial2EnqueueRtCommand (uint8_t c)
{
    return enqueue_realtime_command3(c);
}

static enqueue_realtime_command_ptr serial2SetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command3;

    if(handler)
        enqueue_realtime_command3 = handler;

    return prev;
}

static void __not_in_flash_func(serial2RxCancel) (void)
{
    rx2buf.overflow = false;
    rx2buf.tail = rx2buf.head;
    rx2buf.data[rx2buf.head] = ASCII_CAN;
    rx2buf.head = BUFNEXT(rx2buf.head, rx2buf);
}

static void serial2RxFlush (void)
{
    while(!pio_sm_is_rx_fifo_empty(ppio_uart2.pio_rx, ppio_uart2.sm_rx))
        (void)*((io_rw_8 *)&ppio_uart2.pio_rx->rxf[ppio_uart2.sm_rx] + 3);

    rx2buf.tail = rx2buf.head;
    rx2buf.overflow = false;
}

static bool serial2SetBaudRate (uint32_t baud_rate)
{
    stream_status[1].baud_rate = baud_rate;

    ppio_uart2.char_time_us = (10 * 1000000UL + (baud_rate - 1)) / baud_rate;
    uart_rx_program_set_baud(ppio_uart2.pio_rx, ppio_uart2.sm_rx, baud_rate);
#ifdef UART_2_TX_PIN
    uart_tx_program_set_baud(ppio_uart2.pio_tx, ppio_uart2.sm_tx, baud_rate);
#endif
    return true;
}

static bool serial2SetFormat (serial_format_t format)
{
    (void)format;
    return false;
}

static bool serial2Disable (bool disable)
{
    ppio_uart2.rx_enabled = !disable;

    pio_set_irqn_source_enabled(ppio_uart2.pio_rx, 1, pio_get_rx_fifo_not_empty_interrupt_source(ppio_uart2.sm_rx), !disable);

    return true;
}

static const io_stream_t *serial2Init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .instance = 2,
        .is_connected = stream_connected,
        .read = serial2GetC,
        .enqueue_rt_command = serial2EnqueueRtCommand,
        .get_rx_buffer_free = serial2RxFree,
        .get_rx_buffer_count = serial2RxCount,
        .reset_read_buffer = serial2RxFlush,
        .cancel_read_buffer = serial2RxCancel,
#ifdef UART_2_TX_PIN
        .write = serial2WriteS,
        .write_char = serial2PutC,
        .write_n = serial2Write,
        .get_tx_buffer_count = serial2TxCount,
        .reset_write_buffer = serial2TxFlush,
#endif
        .disable_rx = serial2Disable,
        .suspend_read = serial2SuspendInput,
        .set_baud_rate = serial2SetBaudRate,
        .set_format = NULL, // only 8N1 is supported
        .set_enqueue_rt_handler = serial2SetRtHandler
    };

    io_stream_properties_t *port = get_port(2);
    if(!port->flags.claimable || port->flags.claimed)
        return NULL;

    if(!port->flags.init_ok) {

        bool ok;
        int irq_rx, offset_rx;

        if((ok = pio_claim_free_sm_and_add_program_for_gpio_range(&uart_rx_program, &ppio_uart2.pio_rx, &ppio_uart2.sm_rx, &offset_rx, UART_2_RX_PIN, 1, true))) {
            irq_rx = pio_get_irq_num(ppio_uart2.pio_rx, 1);
            uart_rx_program_init(ppio_uart2.pio_rx, ppio_uart2.sm_rx, offset_rx, UART_2_RX_PIN, baud_rate);
        }
#ifdef UART_2_TX_PIN
        int irq_tx;

        if((ok = ok && pio_claim_free_sm_and_add_program_for_gpio_range(&uart_tx_program, &ppio_uart2.pio_tx, &ppio_uart2.sm_tx, &ppio_uart2.offset_tx, UART_2_TX_PIN, 1, true))) {
            irq_tx = pio_get_irq_num(ppio_uart2.pio_tx, 1);
            uart_tx_program_init(ppio_uart2.pio_tx, ppio_uart2.sm_tx, ppio_uart2.offset_tx, UART_2_TX_PIN, baud_rate);
        }
#endif
        else
            return NULL;

        serial2RxFlush();
        irq_set_exclusive_handler(irq_rx, uart2_interrupt_handler);
        irq_set_enabled(irq_rx, true);
        pio_set_irqn_source_enabled(ppio_uart2.pio_rx, 1, pio_get_rx_fifo_not_empty_interrupt_source(ppio_uart2.sm_rx), true);
#ifdef UART_2_TX_PIN
        if(irq_tx != irq_rx) {
            irq_set_exclusive_handler(irq_tx, uart2_interrupt_handler);
            irq_set_enabled(irq_tx, true);
        }
#endif
        port->flags.init_ok = On;
    }

    port->flags.claimed = On;
    ppio_uart2.rx_enabled = true;

    stream_set_defaults(&stream, baud_rate);

    return &stream;
}

static void __not_in_flash_func(uart2_interrupt_handler)(void)
{
    if(ppio_uart2.rx_enabled) {
        io_rw_8 *rxfifo_shift = (io_rw_8 *)&ppio_uart2.pio_rx->rxf[ppio_uart2.sm_rx] + 3;

        while(!pio_sm_is_rx_fifo_empty(ppio_uart2.pio_rx, ppio_uart2.sm_rx)) {
            uint8_t data = *rxfifo_shift;

            if(!enqueue_realtime_command3(data)) {
                uint_fast16_t next_head = BUFNEXT(rx2buf.head, rx2buf);

                if(next_head == rx2buf.tail)
                    rx2buf.overflow = true;
                else {
                    rx2buf.data[rx2buf.head] = data;
                    rx2buf.head = next_head;
                }
            }
        }
    }

#ifdef UART_2_TX_PIN

    uint_fast16_t tail = tx2buf.tail;

    while(!pio_sm_is_tx_fifo_full(ppio_uart2.pio_tx, ppio_uart2.sm_tx) && tail != tx2buf.head) {
        pio_sm_put(ppio_uart2.pio_tx, ppio_uart2.sm_tx, tx2buf.data[tail]);
        ppio_uart2.tx_deadline = max(ppio_uart2.tx_deadline, time_us_64()) + ppio_uart2.char_time_us;
        tail = BUFNEXT(tail, tx2buf);
    }

    tx2buf.tail = tail;

    if(tx2buf.tail == tx2buf.head)
        pio_set_irqn_source_enabled(ppio_uart2.pio_tx, 1, pio_get_tx_fifo_not_full_interrupt_source(ppio_uart2.sm_tx), false);

#endif
}

#endif // SERIAL2_PORT

