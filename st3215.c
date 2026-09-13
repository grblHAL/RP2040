/*

  st3215.c - plugin for M101, control of a Feetech/Waveshare ST3215 serial bus servo

  Part of grblHAL

  Copyright (c) 2026 Terje Io

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

  Usage:
    M101 [P<id>] [Q<angle>]

  If Q is specified the servo with the given id is moved to <angle> degrees (0-360).
  If Q is omitted the current angle of the servo is reported as [ST3215:<id>|<angle>].
  If P is omitted ST3215_ID_DEFAULT is used.

  The servo is driven over a dedicated hardware UART (ST3215_STREAM, default
  instance 0 = UART0, TX on GPIO0 / RX on GPIO1 unless overridden by the board
  map). ST3215 uses a single-wire half-duplex bus: tie the UART TX and RX pins
  together (a series resistor of a few hundred ohms to 1k on the TX side is the
  usual reference wiring) and connect the joined line to the servo signal pin.
  RX is kept disabled while transmitting so the echo of our own bytes on the
  shared line is not mistaken for a reply.

  Protocol reference: FEETECH SCServo (SMS/STS series) communication protocol.
  Packet: 0xFF 0xFF <id> <length> <instruction> <params...> <checksum>
  checksum = ~(id + length + instruction + params...) & 0xFF
  Reply:   0xFF 0xFF <id> <length> <error> <params...> <checksum>
*/

#include "driver.h"

#if ST3215_ENABLE

#include <string.h>

#include "serial.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"

#ifndef ST3215_STREAM
#define ST3215_STREAM 0 // Hardware UART instance, see serial.c (0 = UART0).
#endif

#ifndef ST3215_BAUDRATE
#define ST3215_BAUDRATE 1000000 // Feetech STS/SMS factory default.
#endif

#ifndef ST3215_ID_DEFAULT
#define ST3215_ID_DEFAULT 1 // Used when M101 is issued without a P<id> word.
#endif

#define ST3215_HEADER           0xFF
#define ST3215_INST_READ        0x02
#define ST3215_INST_WRITE       0x03

#define ST3215_ADDR_TORQUE_ENABLE    40
#define ST3215_ADDR_GOAL_POSITION    42
#define ST3215_ADDR_PRESENT_POSITION 56

#define ST3215_POS_MAX    4095   // Full turn (360 degrees) resolution.
#define ST3215_ANGLE_MAX  360.0f
#define ST3215_TIMEOUT_MS 20

static io_stream_t st3215_uart;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;

static uint8_t st3215_checksum (const uint8_t *buf, uint8_t len)
{
    uint16_t sum = 0;

    while(len--)
        sum += *buf++;

    return (uint8_t)(~sum & 0xFF);
}

// Fire-and-forget WRITE instruction, up to 4 data bytes.
static void st3215_write (uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t packet[10], plen = 0, i;

    packet[plen++] = ST3215_HEADER;
    packet[plen++] = ST3215_HEADER;
    packet[plen++] = id;
    packet[plen++] = (uint8_t)(len + 3); // instruction + addr + data + checksum
    packet[plen++] = ST3215_INST_WRITE;
    packet[plen++] = addr;

    for(i = 0; i < len; i++)
        packet[plen++] = data[i];

    packet[plen] = st3215_checksum(&packet[2], plen - 2);
    plen++;

    st3215_uart.write_n(packet, plen);
    while(st3215_uart.get_tx_buffer_count());
}

// READ instruction, waits for and validates the status reply.
static bool st3215_read (uint8_t id, uint8_t addr, uint8_t len, uint8_t *out)
{
    uint8_t packet[8], plen = 0;
    uint8_t reply[6 + 8];
    uint16_t expected = (uint16_t)(len + 6);
    uint32_t start;

    if(len > 8)
        return false;

    packet[plen++] = ST3215_HEADER;
    packet[plen++] = ST3215_HEADER;
    packet[plen++] = id;
    packet[plen++] = 4; // instruction + addr + len + checksum
    packet[plen++] = ST3215_INST_READ;
    packet[plen++] = addr;
    packet[plen++] = len;
    packet[plen] = st3215_checksum(&packet[2], plen - 2);
    plen++;

    st3215_uart.write_n(packet, plen);
    while(st3215_uart.get_tx_buffer_count());

    st3215_uart.reset_read_buffer();
    st3215_uart.disable_rx(false);

    start = hal.get_elapsed_ticks();
    while(st3215_uart.get_rx_buffer_count() < expected) {
        if(hal.get_elapsed_ticks() - start > ST3215_TIMEOUT_MS)
            break;
    }

    st3215_uart.disable_rx(true);

    if(st3215_uart.get_rx_buffer_count() < expected)
        return false;

    for(uint16_t i = 0; i < expected; i++)
        reply[i] = (uint8_t)st3215_uart.read();

    if(reply[0] != ST3215_HEADER || reply[1] != ST3215_HEADER || reply[2] != id)
        return false;

    if(st3215_checksum(&reply[2], (uint8_t)(expected - 3)) != reply[expected - 1])
        return false;

    memcpy(out, &reply[5], len);

    return true;
}

static bool st3215_set_angle (uint8_t id, float angle)
{
    uint8_t torque_on = 1;
    uint8_t data[2];
    int32_t pos = (int32_t)((angle / ST3215_ANGLE_MAX) * ST3215_POS_MAX + 0.5f);

    if(pos < 0)
        pos = 0;
    if(pos > ST3215_POS_MAX)
        pos = ST3215_POS_MAX;

    st3215_write(id, ST3215_ADDR_TORQUE_ENABLE, &torque_on, 1);

    data[0] = (uint8_t)(pos & 0xFF);
    data[1] = (uint8_t)((pos >> 8) & 0xFF);

    st3215_write(id, ST3215_ADDR_GOAL_POSITION, data, 2);

    return true;
}

static bool st3215_get_angle (uint8_t id, float *angle)
{
    uint8_t data[2];

    if(!st3215_read(id, ST3215_ADDR_PRESENT_POSITION, 2, data))
        return false;

    *angle = (float)((uint16_t)data[0] | ((uint16_t)data[1] << 8)) * ST3215_ANGLE_MAX / (float)ST3215_POS_MAX;

    return true;
}

static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return mcode == UserMCode_Generic1
                     ? UserMCode_Normal
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t mcode_validate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    if(gc_block->user_mcode == UserMCode_Generic1) {

        if(gc_block->words.p && (!isintf(gc_block->values.p) || gc_block->values.p < 0.0f || gc_block->values.p > 253.0f))
            state = Status_GcodeValueOutOfRange;

        if(state == Status_OK && gc_block->words.q && (gc_block->values.q < 0.0f || gc_block->values.q > ST3215_ANGLE_MAX))
            state = Status_GcodeValueOutOfRange;

        gc_block->words.p = gc_block->words.q = Off;

    } else
        state = Status_Unhandled;

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    if(gc_block->user_mcode == UserMCode_Generic1) {

        uint8_t id = gc_block->words.p ? (uint8_t)gc_block->values.p : ST3215_ID_DEFAULT;

        if(gc_block->words.q)
            st3215_set_angle(id, gc_block->values.q);
        else {

            float angle;
            char buf[40];

            if(st3215_get_angle(id, &angle)) {
                strcpy(buf, "[ST3215:");
                strcat(buf, uitoa(id));
                strcat(buf, "|");
                strcat(buf, ftoa(angle, 2));
                strcat(buf, "]" ASCII_EOL);
                hal.stream.write(buf);
            }
        }

    } else if(user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("ST3215 servo", "0.01");
}

void st3215_init (void)
{
    const io_stream_t *stream;

    if((stream = stream_open_instance(ST3215_STREAM, ST3215_BAUDRATE, NULL, "ST3215 servo")) == NULL)
        stream = stream_null_init(ST3215_BAUDRATE);

    memcpy(&st3215_uart, stream, sizeof(io_stream_t));

    st3215_uart.disable_rx(true);
    st3215_uart.set_enqueue_rt_handler(stream_buffer_all);

    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = mcode_check;
    grbl.user_mcode.validate = mcode_validate;
    grbl.user_mcode.execute = mcode_execute;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
}

#endif // ST3215_ENABLE
