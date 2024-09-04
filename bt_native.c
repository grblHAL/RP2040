/*
  bt_native.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Bluetooth comms

  Part of grblHAL

  Copyright (c) 2023 Terje Io

  Some parts of the code is based on example code by Espressif, in the public domain

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

#include "driver.h"

#if BLUETOOTH_ENABLE == 1

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include <btstack.h>
#include <pico/cyw43_arch.h>

#include "grbl/nvs.h"

#include "bt_native.h"
#include "grbl/grbl.h"
#include "grbl/report.h"
#include "grbl/nvs_buffer.h"
#include "grbl/protocol.h"

#define USE_BT_MUTEX 0

#if USE_BT_MUTEX
#define BT_MUTEX_LOCK()   do {} while (xSemaphoreTake(lock, portMAX_DELAY) != pdPASS)
#define BT_MUTEX_UNLOCK() xSemaphoreGive(lock)
static SemaphoreHandle_t lock = NULL;
#else
#define BT_MUTEX_LOCK()
#define BT_MUTEX_UNLOCK()
#endif

typedef struct {
    bool connected;
    uint16_t channel;
    uint8_t status;
    char client_mac[18];
} bt_session_t;

const int RFCOMM_SERVER_CHANNEL = 1;

static const io_stream_t *claim_stream (uint32_t baud_rate);

static bool is_up = false;
static bluetooth_settings_t bluetooth;
static stream_rx_buffer_t rxbuffer = {0};
static stream_tx_buffer_t txbuffer;
static nvs_address_t nvs_address;
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;
static bt_session_t session;
static uint8_t spp_service_buffer[150];
static io_stream_properties_t bt_stream = {
  .type = StreamType_Bluetooth,
  .instance = 20,
  .flags.claimable = On,
  .flags.claimed = Off,
  .flags.connected = Off,
  .flags.can_set_baud = On,
  .flags.modbus_ready = Off,
  .claim = claim_stream
};
static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime;

static enqueue_realtime_command_ptr BTSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

static uint32_t BTStreamAvailable (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t BTStreamRXFree (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static int16_t BTStreamGetC (void)
{
    BT_MUTEX_LOCK();

    int16_t data;
    uint16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head) {
        BT_MUTEX_UNLOCK();
        return -1; // no data available else EOF
    }

    data = rxbuffer.data[bptr++];                 // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer

    BT_MUTEX_UNLOCK();

    return data;
}

// Since grbl always sends cr/lf terminated strings we try to send complete strings to improve throughput
static bool BTStreamPutC (const char c)
{
    uint_fast16_t next_head = BUFNEXT(txbuffer.head, txbuffer);

    while(txbuffer.tail == next_head) {         // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    txbuffer.data[txbuffer.head] = c;           // Add data to buffer
    txbuffer.head = next_head;                  // and update head pointer

    if(c == ASCII_LF && session.connected)
        rfcomm_request_can_send_now_event(session.channel);

    return true;
}

static void BTStreamWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        BTStreamPutC(c);
}

static void BTStreamFlush (void)
{
    BT_MUTEX_LOCK();

    rxbuffer.tail = rxbuffer.head;

    BT_MUTEX_UNLOCK();
}

static int16_t btTxGetC (void)
{
    int16_t data;

    if(txbuffer.tail == txbuffer.head)
        return -1; // no data available else EOF

    data = txbuffer.data[txbuffer.tail];                          // Get next character
    txbuffer.tail = BUFNEXT(txbuffer.tail, txbuffer);  // and update pointer

    return data;
}

static void BTStreamCancel (void)
{
    BT_MUTEX_LOCK();

    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);

    BT_MUTEX_UNLOCK();
}

char *bluetooth_get_device_mac (void)
{
    static char device_mac[18];

    if(is_up) {
        bd_addr_t addr;
        gap_local_bd_addr(addr);
        strcpy(device_mac, bd_addr_to_str(addr));
    } else
        strcpy(device_mac, "-");

    return device_mac;
}

char *bluetooth_get_client_mac (void)
{
    return *session.client_mac ? session.client_mac : NULL;
}

static void report_bt_MAC (bool newopt)
{
    char *client_mac;

    on_report_options(newopt);

    if(newopt)
        hal.stream.write(",BT");
    else {
        hal.stream.write("[BT DEVICE MAC:");
        hal.stream.write(bluetooth_get_device_mac());
        hal.stream.write("]" ASCII_EOL);

        if((client_mac = bluetooth_get_client_mac())) {
            hal.stream.write("[BT CLIENT MAC:");
            hal.stream.write(client_mac);
            hal.stream.write("]" ASCII_EOL);
        }
    }
}

static void lockBluetooth()
{
    async_context_acquire_lock_blocking(cyw43_arch_async_context());
}

static void unlockBluetooth()
{
    async_context_release_lock(cyw43_arch_async_context());
}

static void msg_bt_open_failed (sys_state_t state)
{
    hal.stream.write_all("[MSG:BT RFCOMM channel open failed, status ");
    hal.stream.write_all(uitoa(session.status));
    hal.stream.write_all("]" ASCII_EOL);
}

#if PICO_CYW43_ARCH_POLL && !WIFI_ENABLE

static void bt_poll (sys_state_t state)
{
    static uint32_t last_ms0;

    uint32_t ms = hal.get_elapsed_ticks();

    if(last_ms0 != ms) {
        last_ms0 = ms;
        cyw43_arch_poll();
    }

    on_execute_realtime(state);
}

#endif

static bool is_connected (void)
{
    return bt_stream.flags.connected;
}

static const io_stream_t *claim_stream (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Bluetooth,
        .is_connected = is_connected,
        .read = BTStreamGetC,
        .write = BTStreamWriteS,
        .write_char = BTStreamPutC,
        .get_rx_buffer_free = BTStreamRXFree,
        .reset_read_buffer = BTStreamFlush,
        .cancel_read_buffer = BTStreamCancel,
        .set_enqueue_rt_handler = BTSetRtHandler
    };

    if(bt_stream.flags.claimed || bt_stream.flags.connected)
        return NULL;

    if(baud_rate != 0)
        bt_stream.flags.claimed = On;

    return &stream;
}

static void packetHandler (uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    static uint16_t mtu;
    static uint8_t tx_buf[TX_BUFFER_SIZE];
    static const io_stream_t *stream = NULL;

    UNUSED(channel);

    bd_addr_t event_addr;

    switch (type) {

        case HCI_EVENT_PACKET:
 
            switch (hci_event_packet_get_type(packet)) {

                case HCI_EVENT_PIN_CODE_REQUEST:
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    session.channel = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    rfcomm_accept_connection(session.channel);
                    strcpy(session.client_mac, bd_addr_to_str(event_addr));
                    break;

                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if((session.status = rfcomm_event_channel_opened_get_status(packet)))
                        protocol_enqueue_rt_command(msg_bt_open_failed);
                    else {
                        if(!bt_stream.flags.connected) {

                            rxbuffer.tail = rxbuffer.head;  // Flush rx & tx
                            txbuffer.tail = txbuffer.head;  // buffers.

                            if(bt_stream.flags.claimed)
                                bt_stream.flags.connected = On;
                            else if(!bt_stream.flags.connected && (stream = claim_stream(0)))
                                bt_stream.flags.connected = stream_connect(stream);

                            mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                            session.channel = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                            session.connected = bt_stream.flags.connected;
                        } // else deny connection...
                    
                        // if(!bt_stream.flags.connected) deny connection...
                    }
                    break;
                
                case RFCOMM_EVENT_CAN_SEND_NOW:
                    {
                        int16_t c;
                        uint16_t len = 0;
                        uint8_t *buf = tx_buf;

                        while(len <= mtu && (c = btTxGetC()) != -1) {
                            *buf++ = (uint8_t)c;
                            len++;
                        }

                        if(len)
                            rfcomm_send(session.channel, tx_buf, len);

                        if(c != -1)
                            rfcomm_request_can_send_now_event(session.channel);
                    }
                    break;

                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    session.channel = 0;
                    session.connected = false;
                    *session.client_mac = '\0';
                    if(stream) {
                        stream_disconnect(stream);
                        stream = NULL;
                    }
                    bt_stream.flags.connected = Off;
                    break;

                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:;
            char c;
            while(size) {
                c = (char)*packet++;
                // discard input if MPG has taken over...
                if(hal.stream.type != StreamType_MPG) {

                    if(!enqueue_realtime_command(c)) {

                        uint_fast16_t bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

                        if(bptr == rxbuffer.tail)               // If buffer full
                            rxbuffer.overflow = 1;              // flag overflow,
                        else {
                            rxbuffer.data[rxbuffer.head] = c;   // else add data to buffer
                            rxbuffer.head = bptr;               // and update pointer
                        }
                    }
                }
                size--;
            }
            break;

        default:
            break;
    }
}

bool bluetooth_start_local (void)
{
    static char device_name[52];
    static btstack_packet_callback_registration_t hci_event_callback;
    static io_stream_details_t streams = {
        .n_streams = 1,
        .streams = &bt_stream,
    };

    if(*bluetooth.device_name == '\0')
        return false;

    strcat(strcpy(device_name, bluetooth.device_name), " 00:00:00:00:00:00");

#if !WIFI_ENABLE
    cyw43_arch_init();
#endif

    hci_event_callback.callback = packetHandler;
    hci_add_event_handler(&hci_event_callback);

    l2cap_init();

#ifdef ENABLE_BLE
    sm_init();
#endif

    rfcomm_init();
    rfcomm_register_service(packetHandler, RFCOMM_SERVER_CHANNEL, 0xffff);

    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    spp_create_sdp_record(spp_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL, bluetooth.service_name);
    sdp_register_service(spp_service_buffer);

    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    gap_set_local_name(device_name);

    hci_power_control(HCI_POWER_ON);

    stream_register_streams(&streams);

    is_up = true;

#if PICO_CYW43_ARCH_POLL && !WIFI_ENABLE
    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = bt_poll;
#endif

    return is_up;
}

bool bluetooth_disable (void)
{
    is_up = false;

    return true;
}

static const setting_group_detail_t bluetooth_groups [] = {
    { Group_Root, Group_Bluetooth, "Bluetooth"},
};

static const setting_detail_t bluetooth_settings[] = {
    { Setting_BlueToothDeviceName, Group_Bluetooth, "Bluetooth device name", NULL, Format_String, "x(32)", NULL, "32", Setting_NonCore, bluetooth.device_name, NULL, NULL },
    { Setting_BlueToothServiceName, Group_Bluetooth, "Bluetooth service name", NULL, Format_String, "x(32)", NULL, "32", Setting_NonCore, bluetooth.service_name, NULL, NULL }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t bluetooth_settings_descr[] = {
    { Setting_BlueToothDeviceName, "Bluetooth device name." },
    { Setting_BlueToothServiceName, "Bluetooth service name." },
};

#endif

PROGMEM static const status_detail_t status_detail[] = {
   { Status_BTInitError, "Bluetooth initialisation failed." }
};

static error_details_t error_details = {
    .errors = status_detail,
    .n_errors = sizeof(status_detail) / sizeof(status_detail_t)
};

static void bluetooth_settings_restore (void)
{
    strcpy(bluetooth.device_name, BLUETOOTH_DEVICE);
    strcpy(bluetooth.service_name, BLUETOOTH_SERVICE);

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&bluetooth, sizeof(bluetooth_settings_t), true);
}

static void bluetooth_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&bluetooth, nvs_address, sizeof(bluetooth_settings_t), true) != NVS_TransferResult_OK)
        bluetooth_settings_restore();
}

static void bluetooth_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&bluetooth, sizeof(bluetooth_settings_t), true);
}

static setting_details_t setting_details = {
    .groups = bluetooth_groups,
    .n_groups = sizeof(bluetooth_groups) / sizeof(setting_group_detail_t),
    .settings = bluetooth_settings,
    .n_settings = sizeof(bluetooth_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = bluetooth_settings_descr,
    .n_descriptions = sizeof(bluetooth_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = bluetooth_settings_save,
    .load = bluetooth_settings_load,
    .restore = bluetooth_settings_restore
};

bool bluetooth_init_local (void)
{
    if((nvs_address = nvs_alloc(sizeof(bluetooth_settings_t)))) {

        hal.driver_cap.bluetooth = On;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_bt_MAC;

        errors_register(&error_details);
        settings_register(&setting_details);
    }

    return nvs_address != 0;
}

#endif
