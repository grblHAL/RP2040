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

  If Q is specified the servo with the given id is moved to <angle> degrees.
  If Q is omitted the servo status is reported instead:
    [ST3215:<id>|A:<angle deg>|L:<load %>|V:<voltage V>|T:<temperature C>]
  If P is omitted ST3215_ID_DEFAULT is used.

  The default id's angle is also appended to every "?" realtime status
  report as |ST3215:<angle>, e.g. <Idle|MPos:...|ST3215:120.40>. This value
  is refreshed by a periodic background poll (ST3215_POLL_MS), not read
  live on every "?", so it doesn't add UART round-trip latency to status
  reports.

  $-settings (shared by all servo ids, see $$):
    $450 - move speed, raw ST3215 steps/s (0 = max/uncontrolled speed).
    $451 - minimum allowed angle, degrees. M101 Q below this is rejected.
    $452 - maximum allowed angle, degrees. M101 Q above this is rejected.
  These are stored in NVS and reloaded on boot.

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
#include "grbl/nvs_buffer.h"
#include "grbl/task.h"

#ifndef ST3215_STREAM
#define ST3215_STREAM 0 // Hardware UART instance, see serial.c (0 = UART0).
#endif

#ifndef ST3215_BAUDRATE
#define ST3215_BAUDRATE 1000000 // Feetech STS/SMS factory default.
#endif

#ifndef ST3215_ID_DEFAULT
#define ST3215_ID_DEFAULT 1 // Used when M101 is issued without a P<id> word.
#endif

#ifndef ST3215_SPEED_DEFAULT
#define ST3215_SPEED_DEFAULT 200 // Default $450 value, raw steps/s. Conservative/slow.
#endif

#ifndef ST3215_POLL_MS
#define ST3215_POLL_MS 500 // Background poll interval for the "?" status report field.
#endif

#define ST3215_HEADER           0xFF
#define ST3215_INST_READ        0x02
#define ST3215_INST_WRITE       0x03

#define ST3215_ADDR_TORQUE_ENABLE    40
#define ST3215_ADDR_GOAL_POSITION    42
#define ST3215_ADDR_PRESENT_POSITION 56 // Present Position(2)/Speed(2)/Load(2)/Voltage(1)/Temperature(1), addr 56-63.

#define ST3215_STATUS_LEN 8 // Bytes covering Position..Temperature, read in one transaction.

#define ST3215_POS_MAX     4095   // Full turn (360 degrees) resolution.
#define ST3215_SPEED_MAX   4095   // Raw ST3215 Goal Speed register max (steps/s).
#define ST3215_ANGLE_MAX   360.0f
#define ST3215_TIMEOUT_MS  20

typedef struct {
    float angle;         // degrees
    float load;           // signed percent of max torque, -100.0 .. 100.0
    float voltage;         // volts
    uint8_t temperature;    // degrees C
} st3215_status_t;

// $450-$452, see grbl/settings.h - reserved for private/local plugins.
#define Setting_ST3215_Speed     Setting_UserDefined_0
#define Setting_ST3215_AngleMin  Setting_UserDefined_1
#define Setting_ST3215_AngleMax  Setting_UserDefined_2

typedef struct {
    uint16_t speed;
    float angle_min;
    float angle_max;
} st3215_settings_t;

static io_stream_t st3215_uart;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static on_realtime_report_ptr on_realtime_report;
static nvs_address_t nvs_address;
static st3215_settings_t st3215_settings;

static uint8_t st3215_checksum (const uint8_t *buf, uint8_t len)
{
    uint16_t sum = 0;

    while(len--)
        sum += *buf++;

    return (uint8_t)(~sum & 0xFF);
}

// Fire-and-forget WRITE instruction, up to 8 data bytes.
static void st3215_write (uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t packet[16], plen = 0, i;

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
    uint8_t data[6];
    int32_t pos = (int32_t)((angle / ST3215_ANGLE_MAX) * ST3215_POS_MAX + 0.5f);

    if(pos < 0)
        pos = 0;
    if(pos > ST3215_POS_MAX)
        pos = ST3215_POS_MAX;

    st3215_write(id, ST3215_ADDR_TORQUE_ENABLE, &torque_on, 1);

    // Goal Position (42-43), Goal Time (44-45, left at 0 so Goal Speed governs
    // the move instead), Goal Speed (46-47) - written together in one packet.
    data[0] = (uint8_t)(pos & 0xFF);
    data[1] = (uint8_t)((pos >> 8) & 0xFF);
    data[2] = 0;
    data[3] = 0;
    data[4] = (uint8_t)(st3215_settings.speed & 0xFF);
    data[5] = (uint8_t)((st3215_settings.speed >> 8) & 0xFF);

    st3215_write(id, ST3215_ADDR_GOAL_POSITION, data, 6);

    return true;
}

static bool st3215_get_status (uint8_t id, st3215_status_t *status)
{
    uint8_t data[ST3215_STATUS_LEN];
    uint16_t pos, load_raw;

    if(!st3215_read(id, ST3215_ADDR_PRESENT_POSITION, ST3215_STATUS_LEN, data))
        return false;

    pos = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    status->angle = (float)pos * ST3215_ANGLE_MAX / (float)ST3215_POS_MAX;

    // Present Load (offset 4-5): bits 0-9 magnitude (1000 = 100%), bit 10 direction/sign.
    load_raw = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    status->load = (float)(load_raw & 0x3FF) / 10.0f;
    if(load_raw & 0x400)
        status->load = -status->load;

    status->voltage = (float)data[6] / 10.0f; // Present Voltage (offset 6): units of 0.1V.
    status->temperature = data[7];            // Present Temperature (offset 7): degrees C.

    return true;
}

// Lean 2-byte read, used by the background poll so it doesn't tie up the
// shared UART as long as the full 8-byte st3215_get_status() read.
static bool st3215_get_angle (uint8_t id, float *angle)
{
    uint8_t data[2];

    if(!st3215_read(id, ST3215_ADDR_PRESENT_POSITION, 2, data))
        return false;

    *angle = (float)((uint16_t)data[0] | ((uint16_t)data[1] << 8)) * ST3215_ANGLE_MAX / (float)ST3215_POS_MAX;

    return true;
}

static float st3215_report_angle = 0.0f;
static bool st3215_report_valid = false;

static void st3215_poll (void *data)
{
    float angle;

    if((st3215_report_valid = st3215_get_angle(ST3215_ID_DEFAULT, &angle)))
        st3215_report_angle = angle;

    task_add_delayed(st3215_poll, NULL, ST3215_POLL_MS);
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

        if(state == Status_OK && gc_block->words.q &&
            (gc_block->values.q < st3215_settings.angle_min || gc_block->values.q > st3215_settings.angle_max))
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

            st3215_status_t status;
            char buf[64];

            if(st3215_get_status(id, &status)) {
                strcpy(buf, "[ST3215:");
                strcat(buf, uitoa(id));
                strcat(buf, "|A:");
                strcat(buf, ftoa(status.angle, 2));
                strcat(buf, "|L:");
                strcat(buf, ftoa(status.load, 1));
                strcat(buf, "|V:");
                strcat(buf, ftoa(status.voltage, 1));
                strcat(buf, "|T:");
                strcat(buf, uitoa(status.temperature));
                strcat(buf, "]" ASCII_EOL);
                hal.stream.write(buf);
            }
        }

    } else if(user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static status_code_t set_speed (setting_id_t id, uint_fast16_t value)
{
    st3215_settings.speed = (uint16_t)value;

    return Status_OK;
}

static uint32_t get_speed (setting_id_t id)
{
    return st3215_settings.speed;
}

static status_code_t set_angle_limit (setting_id_t id, float value)
{
    if(id == Setting_ST3215_AngleMin)
        st3215_settings.angle_min = value;
    else
        st3215_settings.angle_max = value;

    return Status_OK;
}

static float get_angle_limit (setting_id_t id)
{
    return id == Setting_ST3215_AngleMin ? st3215_settings.angle_min : st3215_settings.angle_max;
}

static const setting_detail_t st3215_settings_detail[] = {
    { Setting_ST3215_Speed, Group_General, "ST3215 servo speed", "steps/s", Format_Int16, "####0", "0", "4095", Setting_IsExtendedFn, set_speed, get_speed, NULL },
    { Setting_ST3215_AngleMin, Group_General, "ST3215 minimum angle", "deg", Format_Decimal, "##0.0", "0", "360", Setting_IsExtendedFn, set_angle_limit, get_angle_limit, NULL },
    { Setting_ST3215_AngleMax, Group_General, "ST3215 maximum angle", "deg", Format_Decimal, "##0.0", "0", "360", Setting_IsExtendedFn, set_angle_limit, get_angle_limit, NULL }
};

static const setting_descr_t st3215_settings_descr[] = {
    { Setting_ST3215_Speed, "Move speed for M101 Q<angle>, in raw ST3215 Goal Speed units (steps/s out of 4096 per revolution). 0 = maximum/uncontrolled speed." },
    { Setting_ST3215_AngleMin, "Minimum angle allowed for M101 Q<angle>. Moves requesting a lower angle are rejected." },
    { Setting_ST3215_AngleMax, "Maximum angle allowed for M101 Q<angle>. Moves requesting a higher angle are rejected." }
};

static void st3215_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&st3215_settings, sizeof(st3215_settings_t), true);
}

static void st3215_settings_restore (void)
{
    st3215_settings.speed = ST3215_SPEED_DEFAULT;
    st3215_settings.angle_min = 0.0f;
    st3215_settings.angle_max = ST3215_ANGLE_MAX;

    st3215_settings_save();
}

static void st3215_settings_load (void)
{
    bool valid;

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&st3215_settings, nvs_address, sizeof(st3215_settings_t), true) != NVS_TransferResult_OK)
        st3215_settings_restore();

    // Guard against a corrupted NVS sector (e.g. disturbed by a firmware reflash)
    // silently arming a bogus angle range.
    valid = st3215_settings.speed <= ST3215_SPEED_MAX &&
            st3215_settings.angle_min >= 0.0f && st3215_settings.angle_min <= ST3215_ANGLE_MAX &&
            st3215_settings.angle_max >= 0.0f && st3215_settings.angle_max <= ST3215_ANGLE_MAX &&
            st3215_settings.angle_min <= st3215_settings.angle_max;

    if(!valid)
        st3215_settings_restore();
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(st3215_report_valid) {
        char buf[24];
        strcpy(buf, "|ST3215:");
        strcat(buf, ftoa(st3215_report_angle, 2));
        stream_write(buf);
    }

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("ST3215 servo", "0.03");
}

void st3215_init (void)
{
    static setting_details_t setting_details = {
        .settings = st3215_settings_detail,
        .n_settings = sizeof(st3215_settings_detail) / sizeof(setting_detail_t),
        .descriptions = st3215_settings_descr,
        .n_descriptions = sizeof(st3215_settings_descr) / sizeof(setting_descr_t),
        .save = st3215_settings_save,
        .load = st3215_settings_load,
        .restore = st3215_settings_restore
    };

    const io_stream_t *stream;

    if((stream = stream_open_instance(ST3215_STREAM, ST3215_BAUDRATE, NULL, "ST3215 servo")) == NULL)
        stream = stream_null_init(ST3215_BAUDRATE);

    memcpy(&st3215_uart, stream, sizeof(io_stream_t));

    st3215_uart.disable_rx(true);
    st3215_uart.set_enqueue_rt_handler(stream_buffer_all);

    if((nvs_address = nvs_alloc(sizeof(st3215_settings_t))))
        settings_register(&setting_details);

    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = mcode_check;
    grbl.user_mcode.validate = mcode_validate;
    grbl.user_mcode.execute = mcode_execute;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_realtime_report = grbl.on_realtime_report;
    grbl.on_realtime_report = onRealtimeReport;

    task_add_delayed(st3215_poll, NULL, ST3215_POLL_MS);
}

#endif // ST3215_ENABLE
