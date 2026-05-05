/*
  wifi.h - An embedded CNC Controller with rs274/ngc (g-code) support

   WiFi comms for RP2040 (Pi Pico W)

  Part of grblHAL

  Copyright (c) 2023 Terje Io

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

#ifndef _grbl_wifi_h_
#define _grbl_wifi_h_

#include "driver.h"
#include "lwip/ip_addr.h"

#ifndef STREAM_POLL_INTERVAL
#define STREAM_POLL_INTERVAL 2 // Poll interval in milliseconds
#endif

#define WIFI_AUTH_OPEN 0

typedef struct {
    uint8_t bssid[6];   ///< access point mac address
    ssid_t ssid;        ///< wlan access point name
    bool primary;
    uint32_t authmode;  ///< wifi auth mode \ref CYW43_AUTH_
    int16_t rssi;       ///< signal strength
    uint16_t channel;
} ap_record_t;

typedef struct {
    uint16_t ap_num;
    uint32_t timestamp;
    ap_record_t *ap_records;
    uint8_t *ap_selected;
    ip4_addr_t ip_addr;
    char ap_status[20];
} ap_list_t;

bool wifi_init (void);
bool wifi_start (void);
bool wifi_stop (void);
bool wifi_ap_connect (char *ssid, char *password);
void wifi_ap_scan (void);
ap_list_t *wifi_get_aplist (void);
void wifi_release_aplist (void);
char *wifi_get_authmode_name (uint32_t authmode);

#endif

