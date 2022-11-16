/*
  wifi.c - An embedded CNC Controller with rs274/ngc (g-code) support

  WiFi comms for RP2040 (Pi Pico W)

  Part of grblHAL

  Copyright (c) 2022 Terje Io

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

#if WIFI_ENABLE

#include <string.h>

 #include "pico/stdlib.h"
 #include "pico/cyw43_arch.h"

#include "networking/networking.h"
#include "networking/utils.h"
#include "lwip/timeouts.h"
#include "lwip/apps/mdns.h"

#include "wifi.h"
#include "dhcpserver.h"
#include "grbl/report.h"
#include "grbl/nvs_buffer.h"
#include "grbl/protocol.h"

typedef struct
{
    grbl_wifi_mode_t mode;
    wifi_sta_settings_t sta;
    wifi_ap_settings_t ap;
} wifi_settings_t;

static int interface;
static volatile bool linkUp = false;
static bool scan_in_progress = false;
static char IPAddress[IP4ADDR_STRLEN_MAX];
static stream_type_t active_stream = StreamType_Null;
static wifi_settings_t wifi;
static network_settings_t network;
static network_services_t services = {0}, allowed_services;
static ap_list_t ap_list = {0}, ap_list_found = {0};
static dhcp_server_t dhcp_server;
static nvs_address_t nvs_address;
static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime;
static on_stream_changed_ptr on_stream_changed;
static char netservices[NETWORK_SERVICES_LEN] = ""; // must be large enough to hold all service names

ap_list_t *wifi_get_aplist (void)
{
    if(ap_list_found.ap_records)
        return &ap_list_found;
    else
        return NULL;
}

void wifi_release_aplist (void)
{
    if(ap_list_found.ap_records) {
        free(ap_list_found.ap_records);
        ap_list_found.ap_records = NULL;
    }
}

char *wifi_get_authmode_name (uint32_t authmode)
{
    return authmode == CYW43_AUTH_OPEN ? "open" :
           authmode == CYW43_AUTH_WPA_TKIP_PSK ? "wpa-psk" :
           authmode == CYW43_AUTH_WPA2_AES_PSK ? "wpa2-psk" :
           authmode == CYW43_AUTH_WPA2_MIXED_PSK ? "wpa-wpa2-psk" :
           "unknown";
}

char *wifi_get_ipaddr (void)
{
   return IPAddress;
}

char *wifi_get_mac (void)
{
    static char mac[18];
    uint8_t bmac[6];

    cyw43_wifi_get_mac(&cyw43_state, interface, bmac);
    sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X", bmac[0], bmac[1], bmac[2], bmac[3], bmac[4], bmac[5]);

    return mac;
}

static void reportIP (bool newopt)
{
    on_report_options(newopt);

    if(newopt) {
#if FTP_ENABLE
        hal.stream.write(",WIFI,FTP");
#else
        hal.stream.write(",WIFI");
#endif
#if WEBDAV_ENABLE
        if(services.webdav)
            hal.stream.write(",WebDAV");
#endif
#if MDNS_ENABLE
        if(services.mdns)
            hal.stream.write(",mDNS");
#endif
#if SSDP_ENABLE
        if(services.ssdp)
            hal.stream.write(",SSDP");
#endif
    } else {
        hal.stream.write("[WIFI MAC:");
        hal.stream.write(wifi_get_mac());
        hal.stream.write("]" ASCII_EOL);

        hal.stream.write("[IP:");
        hal.stream.write(wifi_get_ipaddr());
        hal.stream.write("]" ASCII_EOL);

        if(active_stream == StreamType_Telnet || active_stream == StreamType_WebSocket) {
            hal.stream.write("[NETCON:");
            hal.stream.write(active_stream == StreamType_Telnet ? "Telnet" : "Websocket");
            hal.stream.write("]" ASCII_EOL);
        }
    }
}

network_info_t *networking_get_info (void)
{
    static network_info_t info;

    memcpy(&info.status, &network, sizeof(network_settings_t));

    strcpy(info.mac, wifi_get_mac());
    strcpy(info.status.ip, wifi_get_ipaddr());

    if(info.status.ip_mode == IpMode_DHCP) {
        *info.status.gateway = '\0';
        *info.status.mask = '\0';
    }

    info.is_ethernet = false;
    info.link_up = false;
//    info.mbps = 100;
    info.status.services = services;

    return &info;
}

#if MDNS_ENABLE

static void mdns_device_info (struct mdns_service *service, void *txt_userdata)
{
    char build[20] = "build=";

    strcat(build, uitoa(GRBL_BUILD));
    mdns_resp_add_service_txtitem(service, "model=grblHAL", 13);
    mdns_resp_add_service_txtitem(service, (char *)txt_userdata, strlen((char *)txt_userdata));
    mdns_resp_add_service_txtitem(service, build, strlen(build));
}

static void mdns_service_info (struct mdns_service *service, void *txt_userdata)
{
    if(txt_userdata)
        mdns_resp_add_service_txtitem(service, (char *)txt_userdata, strlen((char *)txt_userdata));
}

#endif

static void lwIPHostTimerHandler (void *arg)
{
    if(services.mask)
        sys_timeout(STREAM_POLL_INTERVAL, lwIPHostTimerHandler, NULL);

#if TELNET_ENABLE
    if(services.telnet)
        telnetd_poll();
#endif
#if WEBSOCKET_ENABLE
    if(services.websocket)
        websocketd_poll();
#endif
#if FTP_ENABLE
    if(services.ftp)
        ftpd_poll();
#endif
}

static void start_services (void)
{
#if TELNET_ENABLE
    if(network.services.telnet && !services.telnet)
        services.telnet = telnetd_init(network.telnet_port);
#endif

#if WEBSOCKET_ENABLE
    if(network.services.websocket && !services.websocket)
        services.websocket = websocketd_init(network.websocket_port);
#endif

#if FTP_ENABLE
    if(network.services.ftp && !services.ftp)
        services.ftp = ftpd_init(network.ftp_port);
#endif

#if HTTP_ENABLE
    if(network.services.http && !services.http) {
        services.http = httpd_init(network.http_port);
  #if WEBDAV_ENABLE
        if(network.services.webdav && !services.webdav)
            services.webdav = webdav_init();
  #endif
  #if SSDP_ENABLE
        if(network.services.ssdp && !services.ssdp)
            services.ssdp = ssdp_init(network.http_port);
  #endif
    }
#endif

#if MDNS_ENABLE
    if(*network.hostname && network.services.mdns && !services.mdns) {

        mdns_resp_init();

        if((services.mdns = mdns_resp_add_netif(netif_default, network.hostname) == ERR_OK)) {

            mdns_resp_add_service(netif_default, network.hostname, "_device-info", DNSSD_PROTO_TCP, 0, mdns_device_info, "version=" GRBL_VERSION);

            if(services.http)
                mdns_resp_add_service(netif_default, network.hostname, "_http", DNSSD_PROTO_TCP, network.http_port, mdns_service_info, "path=/");
            if(services.webdav)
                mdns_resp_add_service(netif_default, network.hostname, "_webdav", DNSSD_PROTO_TCP, network.http_port, mdns_service_info, "path=/");
            if(services.websocket)
                mdns_resp_add_service(netif_default, network.hostname, "_websocket", DNSSD_PROTO_TCP, network.websocket_port, mdns_service_info, NULL);
            if(services.telnet)
                mdns_resp_add_service(netif_default, network.hostname, "_telnet", DNSSD_PROTO_TCP, network.telnet_port, mdns_service_info, NULL);
            if(services.ftp)
                mdns_resp_add_service(netif_default, network.hostname, "_ftp", DNSSD_PROTO_TCP, network.ftp_port, mdns_service_info, "path=/");

//            mdns_resp_announce(netif_default);
        }
    }
#endif

#if TELNET_ENABLE || WEBSOCKET_ENABLE || FTP_ENABLE
    sys_timeout(STREAM_POLL_INTERVAL, lwIPHostTimerHandler, NULL);
#endif
}

static void stop_services (void)
{
    network_services_t running;

    running.mask = services.mask;
    services.mask = 0;

#if xHTTP_ENABLE
    if(running.http)
        httpdaemon_stop();
#endif
#if TELNET_ENABLE
    if(running.telnet)
        telnetd_stop();
#endif
#if WEBSOCKET_ENABLE
    if(running.websocket)
        websocketd_stop();
#endif
#if SSDP_ENABLE
    if(!running.ssdp)
        ssdp_stop();
#endif
//    if(running.dns)
//        dns_server_stop();
}

static void msg_ap_ready (sys_state_t state)
{
    hal.stream.write_all("[MSG:WIFI AP READY]" ASCII_EOL);
}

static void msg_ap_connected (sys_state_t state)
{
    hal.stream.write_all("[MSG:WIFI AP CONNECTED]" ASCII_EOL);
}

static void msg_ap_scan_completed (sys_state_t state)
{
    hal.stream.write_all("[MSG:WIFI AP SCAN COMPLETED]" ASCII_EOL);
}

static void msg_ap_disconnected (sys_state_t state)
{
    hal.stream.write_all("[MSG:WIFI AP DISCONNECTED]" ASCII_EOL);
}

static void msg_sta_active (sys_state_t state)
{
    char buf[50];

    sprintf(buf, "[MSG:WIFI STA ACTIVE, IP=%s]" ASCII_EOL, wifi_get_ipaddr());

    hal.stream.write_all(buf);
}

static void msg_sta_disconnected (sys_state_t state)
{
    hal.stream.write_all("[MSG:WIFI STA DISCONNECTED]" ASCII_EOL);
}

static void msg_sta_failed (sys_state_t state)
{
    hal.stream.write_all("[MSG:WIFI STA CONNECT FAILED]" ASCII_EOL);
}

static void msg_wifi_failed (sys_state_t state)
{
    hal.stream.write_all("[MSG:WIFI STARTUP FAILED]" ASCII_EOL);
}

static int scan_result(void *env, const cyw43_ev_scan_result_t *result)
{
    if (result) {

        if(ap_list.ap_records) { // do not add duplicates
            ssid_t ssid;
            uint_fast8_t idx = ap_list.ap_num;
 
            strncpy(ssid, result->ssid, result->ssid_len);
            ssid[result->ssid_len] = '\0';
 
            do {
                if(!strcmp(ap_list.ap_records[--idx].ssid, ssid))
                    return 0;
            } while(idx);
        }

        ap_record_t *records = realloc((void *)ap_list.ap_records, (ap_list.ap_num + 1) * sizeof(ap_record_t));
        if(records) {
            ap_list.ap_records = records;
            ap_list.ap_records[ap_list.ap_num].authmode = result->auth_mode;
            ap_list.ap_records[ap_list.ap_num].rssi = result->rssi;
            ap_list.ap_records[ap_list.ap_num].primary = true;            
            ap_list.ap_records[ap_list.ap_num].channel = result->channel;
            memcpy(&ap_list.ap_records[ap_list.ap_num].bssid, result->bssid, sizeof(result->bssid));
            strncpy(ap_list.ap_records[ap_list.ap_num].ssid, result->ssid, result->ssid_len);
            ap_list.ap_records[ap_list.ap_num].ssid[result->ssid_len] = '\0';
            ap_list.ap_num++;
        }
    }

    return 0;
}

static void enet_poll (sys_state_t state)
{
    static bool led_on = false;
    static uint32_t last_ms0, next_ms1;

    uint32_t ms = hal.get_elapsed_ticks();

    if(last_ms0 != ms) {
        last_ms0 = ms;
        cyw43_arch_poll();
        linkUp = cyw43_tcpip_link_status(&cyw43_state, interface) == CYW43_LINK_UP;
    }

    if(scan_in_progress && (ms > next_ms1)) {
 
        led_on = !led_on;
        next_ms1 = ms + 1000;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

        if(!(scan_in_progress = cyw43_wifi_scan_active(&cyw43_state))) {
            if(ap_list.ap_records) {
                wifi_release_aplist();
                memcpy(&ap_list_found, &ap_list, sizeof(ap_list_t));
                ap_list_found.timestamp = hal.get_elapsed_ticks();
                memset(&ap_list, 0, sizeof(ap_list_t));
            }
            protocol_enqueue_rt_command(msg_ap_scan_completed);
        }
    }

    on_execute_realtime(state);
}

void wifi_ap_scan (void)
{
    ap_list.ap_num = 0;
    if(ap_list.ap_records) {
        free(ap_list.ap_records);
        ap_list.ap_records = NULL;
    }

    if (!scan_in_progress) {
        cyw43_wifi_scan_options_t scan_options = {0};
        scan_in_progress = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result) == 0;
    } else if (!cyw43_wifi_scan_active(&cyw43_state)) {
        scan_in_progress = false; 
    }
}

/*
define CYW43_LINK_DOWN         (0)     ///< link is down
#define CYW43_LINK_JOIN         (1)     ///< Connected to wifi
#define CYW43_LINK_NOIP         (2)     ///< Connected to wifi, but no IP address
#define CYW43_LINK_UP           (3)     ///< Connect to wifi with an IP address
#define CYW43_LINK_FAIL         (-1)    ///< Connection failed
#define CYW43_LINK_NONET        (-2)    ///< No matching SSID found (could be out of range, or down)
#define CYW43_LINK_BADAUTH      (-3)    ///< Authenticatation failure
*/
static void netif_status_callback (struct netif *netif)
{
    switch(cyw43_tcpip_link_status(&cyw43_state, interface)) {

        case CYW43_LINK_UP:
            if(netif->ip_addr.addr != 0) {
                ip4addr_ntoa_r(netif_ip_addr4(netif), IPAddress, IP4ADDR_STRLEN_MAX);
                start_services();
            }
            protocol_enqueue_rt_command(msg_sta_active);
            break;

        case CYW43_LINK_DOWN:
            *IPAddress = '\0';
            protocol_enqueue_rt_command(msg_sta_disconnected);
            break;

        case CYW43_LINK_FAIL:
        case CYW43_LINK_NONET:
        case CYW43_LINK_BADAUTH:
           *IPAddress = '\0';
            protocol_enqueue_rt_command(msg_sta_failed);
            break;

        default:
            break;
    }
}

static inline void set_addr (char *ip, ip4_addr_t *addr)
{
    memcpy(ip, addr, sizeof(ip4_addr_t));
}

static inline void get_addr (ip4_addr_t *addr, char *ip)
{
    memcpy(addr, ip, sizeof(ip4_addr_t));
}

bool wifi_start (void)
{
    if(nvs_address == 0)
        return false;

    if (cyw43_arch_init()) {
        protocol_enqueue_rt_command(msg_wifi_failed);
        return false;
    }

#ifdef WIFI_SOFTAP 

    if(wifi.mode == WiFiMode_AP) {

        if(*wifi.ap.ssid == '\0')
            return false;

        interface = CYW43_ITF_AP;
        memcpy(&network, &wifi.ap.network, sizeof(network_settings_t));

        if(network.telnet_port == 0)
            network.telnet_port = NETWORK_TELNET_PORT;
        if(network.websocket_port == 0)
            network.websocket_port = NETWORK_WEBSOCKET_PORT;
        if(network.http_port == 0)
            network.http_port = NETWORK_HTTP_PORT;
        if(network.ftp_port == 0)
            network.ftp_port = NETWORK_FTP_PORT;
        
        cyw43_arch_enable_ap_mode(wifi.ap.ssid, wifi.ap.password, CYW43_AUTH_WPA2_AES_PSK);

        netif_set_status_callback(netif_default, netif_status_callback);
//        netif_set_link_callback(netif_default, link_status_callback);

        ip4_addr_t ip, mask;

        get_addr(&ip, network.ip);
        get_addr(&mask, network.mask);

        dhcp_server_init(&dhcp_server, &ip, &mask);
 
        ip4addr_ntoa_r(netif_ip_addr4(netif_default), IPAddress, IP4ADDR_STRLEN_MAX);
        start_services();

        wifi_ap_scan();
    }

#endif

    if(wifi.mode == WiFiMode_STA) {
        
        interface = CYW43_ITF_STA;
        memcpy(&network, &wifi.sta.network, sizeof(network_settings_t));

        if(network.telnet_port == 0)
            network.telnet_port = NETWORK_TELNET_PORT;
        if(network.websocket_port == 0)
            network.websocket_port = NETWORK_WEBSOCKET_PORT;
        if(network.http_port == 0)
            network.http_port = NETWORK_HTTP_PORT;
        if(network.ftp_port == 0)
            network.ftp_port = NETWORK_FTP_PORT;

        cyw43_arch_enable_sta_mode();

        netif_set_status_callback(netif_default, netif_status_callback);
//        netif_set_link_callback(netif_default, link_status_callback);

        if(*wifi.sta.ssid != '\0')
            cyw43_arch_wifi_connect_async(wifi.sta.ssid, wifi.sta.password, CYW43_AUTH_WPA2_AES_PSK);
    }

#if LWIP_NETIF_HOSTNAME
    netif_set_hostname(netif_default, network.hostname);
#endif

#if PICO_CYW43_ARCH_POLL
    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = enet_poll;
#endif

    return true;
}

bool wifi_ap_connect (char *ssid, char *password)
{
    bool ok = !ssid || (strlen(ssid) > 0 && strlen(ssid) < sizeof(ssid_t) && strlen(password) < sizeof(password_t));

    if(!ok)
        return false;
/*
    if(xEventGroupGetBits(wifi_event_group) & CONNECTED_BIT)
        esp_wifi_disconnect(); // TODO: delay until response is sent...

    if(xSemaphoreTake(aplist_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

        ap_list.ap_selected = NULL;
        memset(&ap_list.ip_addr, 0, sizeof(ip4_addr_t));
        strcpy(ap_list.ap_status, ssid ? "Connecting..." : "");

        xSemaphoreGive(aplist_mutex);
    }

    memset(&wifi_sta_config, 0, sizeof(wifi_config_t));

    if(ssid) {

        strcpy((char *)wifi_sta_config.sta.ssid, ssid);
        strcpy((char *)wifi_sta_config.sta.password, password);

        ok = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_sta_config) == ESP_OK && esp_wifi_connect() == ESP_OK;
    }
*/
    return ok;
}

bool wifi_stop (void)
{
    stop_services();

    cyw43_arch_deinit();

    return true;
}

wifi_settings_t *get_wifi_settings (void)
{
    return &wifi;
}

network_settings_t *get_network_settings (void)
{
    return &network;   
}

static status_code_t wifi_set_int (setting_id_t setting, uint_fast16_t value);
static uint_fast16_t wifi_get_int (setting_id_t setting);
static status_code_t wifi_set_ip (setting_id_t setting, char *value);
static char *wifi_get_ip (setting_id_t setting);
static void wifi_settings_restore (void);
static void wifi_settings_load (void);

static const setting_group_detail_t ethernet_groups [] = {
    { Group_Root, Group_Networking, "Networking" },
    { Group_Networking, Group_Networking_Wifi, "WiFi" }
};

static const setting_detail_t ethernet_settings[] = {
    { Setting_NetworkServices, Group_Networking, "Network Services", NULL, Format_Bitfield, netservices, NULL, NULL, Setting_NonCoreFn, wifi_set_int, wifi_get_int, NULL, { .reboot_required = On } },
    { Setting_WiFi_STA_SSID, Group_Networking_Wifi, "WiFi Station (STA) SSID", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, &wifi.sta.ssid, NULL, NULL },
    { Setting_WiFi_STA_Password, Group_Networking_Wifi, "WiFi Station (STA) Password", NULL, Format_Password, "x(32)", NULL, "32", Setting_NonCore, &wifi.sta.password, NULL, NULL, { .allow_null = On } },
    { Setting_Hostname, Group_Networking, "Hostname", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, &wifi.sta.network.hostname, NULL, NULL, { .reboot_required = On } },
/*    { Setting_IpMode, Group_Networking, "IP Mode", NULL, Format_RadioButtons, "Static,DHCP,AutoIP", NULL, NULL, Setting_NonCoreFn, wifi_set_int, wifi_get_int, NULL, { .reboot_required = On } }, */
    { Setting_IpAddress, Group_Networking, "IP Address", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, wifi_set_ip, wifi_get_ip, NULL, { .reboot_required = On } },
    { Setting_Gateway, Group_Networking, "Gateway", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, wifi_set_ip, wifi_get_ip, NULL, { .reboot_required = On } },
    { Setting_NetMask, Group_Networking, "Netmask", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, wifi_set_ip, wifi_get_ip, NULL, { .reboot_required = On } },
#if WIFI_SOFTAP
    { Setting_WifiMode, Group_Networking_Wifi, "WiFi Mode", NULL, Format_RadioButtons, "Off,Station,Access Point", NULL, NULL, Setting_NonCore, &wifi.mode, NULL, NULL },
    { Setting_WiFi_AP_SSID, Group_Networking_Wifi, "WiFi Access Point (AP) SSID", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, &wifi.ap.ssid, NULL, NULL },
    { Setting_WiFi_AP_Password, Group_Networking_Wifi, "WiFi Access Point (AP) Password", NULL, Format_Password, "x(32)", NULL, "32", Setting_NonCore, &wifi.ap.password, NULL, NULL, { .allow_null = On, .reboot_required = On } },
    { Setting_Hostname2, Group_Networking, "Hostname (AP)", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, &wifi.ap.network.hostname, NULL, NULL, { .reboot_required = On } },
    { Setting_IpAddress2, Group_Networking, "IP Address (AP)", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, wifi_set_ip, wifi_get_ip, NULL, { .reboot_required = On } },
//    { Setting_Gateway2, Group_Networking, "Gateway (AP)", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, wifi_set_ip, wifi_get_ip, NULL, { .reboot_required = On } },
    { Setting_NetMask2, Group_Networking, "Netmask (AP)", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, wifi_set_ip, wifi_get_ip, NULL, { .reboot_required = On } },
#else
    { Setting_WifiMode, Group_Networking_Wifi, "WiFi Mode", NULL, Format_RadioButtons, "Off,Station", NULL, NULL, Setting_NonCore, &wifi.mode, NULL, NULL },
#endif
#if TELNET_ENABLE
    { Setting_TelnetPort, Group_Networking, "Telnet port", NULL, Format_Integer, "####0", "1", "65535", Setting_NonCoreFn, wifi_set_int, wifi_get_int, NULL, { .reboot_required = On } },
#endif
#if HTTP_ENABLE
    { Setting_HttpPort, Group_Networking, "HTTP port", NULL, Format_Integer, "####0", "1", "65535", Setting_NonCoreFn, wifi_set_int, wifi_get_int, NULL, { .reboot_required = On } },
#endif
#if FTP_ENABLE
    { Setting_FtpPort, Group_Networking, "FTP port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCoreFn, wifi_set_int, wifi_get_int, NULL, { .reboot_required = On } },
#endif
#if WEBSOCKET_ENABLE
    { Setting_WebSocketPort, Group_Networking, "Websocket port", NULL, Format_Integer, "####0", "1", "65535", Setting_NonCoreFn, wifi_set_int, wifi_get_int, NULL, { .reboot_required = On } }
#endif
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t ethernet_settings_descr[] = {
    { Setting_NetworkServices, "Network services to enable. Consult driver documentation for availability." },
    { Setting_WiFi_STA_SSID, "WiFi Station (STA) SSID." },
    { Setting_WiFi_STA_Password, "WiFi Station (STA) Password." },
    { Setting_Hostname, "Network hostname." },
//    { Setting_IpMode, "IP Mode." },
    { Setting_IpAddress, "Static IP address." },
    { Setting_Gateway, "Static gateway address." },
    { Setting_NetMask, "Static netmask." },
#if WIFI_SOFTAP
    { Setting_WifiMode, "WiFi Mode." },
    { Setting_WiFi_AP_SSID, "WiFi Access Point (AP) SSID." },
    { Setting_WiFi_AP_Password, "WiFi Access Point (AP) Password." },
    { Setting_Hostname2, "Network hostname." },
    { Setting_IpAddress2, "Static IP address." },
    { Setting_Gateway2, "Static gateway address." },
    { Setting_NetMask2, "Static netmask." },
#else
    { Setting_WifiMode, "WiFi Mode." },
#endif
#if TELNET_ENABLE
    { Setting_TelnetPort, "(Raw) Telnet port number listening for incoming connections." },
#endif
#if FTP_ENABLE
    { Setting_FtpPort, "FTP port number listening for incoming connections." },
#endif
#if HTTP_ENABLE
    { Setting_HttpPort, "HTTP port number listening for incoming connections." },
#endif
#if WEBSOCKET_ENABLE
    { Setting_WebSocketPort, "Websocket port number listening for incoming connections."
                             "NOTE: WebUI requires this to be HTTP port number + 1."
    }
#endif
};

#endif

static void wifi_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&wifi, sizeof(wifi_settings_t), true);
}

static setting_details_t setting_details = {
    .groups = ethernet_groups,
    .n_groups = sizeof(ethernet_groups) / sizeof(setting_group_detail_t),
    .settings = ethernet_settings,
    .n_settings = sizeof(ethernet_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = ethernet_settings_descr,
    .n_descriptions = sizeof(ethernet_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = wifi_settings_save,
    .load = wifi_settings_load,
    .restore = wifi_settings_restore
};

static status_code_t wifi_set_int (setting_id_t setting, uint_fast16_t value)
{
    switch(setting) {

        case Setting_NetworkServices:
            wifi.sta.network.services.mask = wifi.ap.network.services.mask = (uint8_t)value & allowed_services.mask;
            break;

#if TELNET_ENABLE
        case Setting_TelnetPort:
            wifi.sta.network.telnet_port = wifi.ap.network.telnet_port = (uint16_t)value;
            break;
#endif

#if FTP_ENABLE
        case Setting_FtpPort:
            wifi.sta.network.ftp_port = wifi.ap.network.ftp_port = (uint16_t)value;
            break;
#endif

#if HTTP_ENABLE
        case Setting_HttpPort:
            wifi.sta.network.http_port = wifi.ap.network.http_port = (uint16_t)value;
            break;
#endif

#if WEBSOCKET_ENABLE
        case Setting_WebSocketPort:
            wifi.sta.network.websocket_port = wifi.ap.network.websocket_port = (uint16_t)value;
            break;
#endif
        default:
            break;
    }

    return Status_OK;
}

static uint_fast16_t wifi_get_int (setting_id_t setting)
{
    uint_fast16_t value = 0;

    switch(setting) {

        case Setting_NetworkServices:
            value = wifi.sta.network.services.mask & allowed_services.mask;
            break;

#if TELNET_ENABLE
        case Setting_TelnetPort:
            value = wifi.sta.network.telnet_port;
            break;
#endif

#if FTP_ENABLE
        case Setting_FtpPort:
            value = wifi.sta.network.ftp_port;
            break;
#endif

#if HTTP_ENABLE
        case Setting_HttpPort:
            value = wifi.sta.network.http_port;
            break;
#endif

#if WEBSOCKET_ENABLE
        case Setting_WebSocketPort:
            value = wifi.sta.network.websocket_port;
            break;
#endif
        default:
            break;
    }

    return value;
}

static status_code_t wifi_set_ip (setting_id_t setting, char *value)
{
    ip4_addr_t addr;

    if(ip4addr_aton(value, &addr) != 1)
        return Status_InvalidStatement;

    status_code_t status = Status_OK;

    switch(setting) {

        case Setting_IpAddress:
            set_addr(wifi.sta.network.ip, &addr);
            break;

        case Setting_Gateway:
            set_addr(wifi.sta.network.gateway, &addr);
            break;

        case Setting_NetMask:
            set_addr(wifi.sta.network.mask, &addr);
            break;

#if WIFI_SOFTAP

        case Setting_IpAddress2:
            if(strcmp("192.168.4.1", value))
                status = Status_SettingDisabled;
            else
                set_addr(wifi.ap.network.ip, &addr);
            break;

        case Setting_Gateway2:
            set_addr(wifi.ap.network.gateway, &addr);
            break;

        case Setting_NetMask2:
            set_addr(wifi.ap.network.mask, &addr);
            break;

#endif

        default:
            status = Status_Unhandled;
            break;
    }

    return status;
}

static char *wifi_get_ip (setting_id_t setting)
{
    static char ip[IPADDR_STRLEN_MAX];

    switch(setting) {

        case Setting_IpAddress:
            ip4addr_ntoa_r((const ip_addr_t *)&wifi.sta.network.ip, ip, IPADDR_STRLEN_MAX);
            break;

        case Setting_Gateway:
            ip4addr_ntoa_r((const ip_addr_t *)&wifi.sta.network.gateway, ip, IPADDR_STRLEN_MAX);
            break;

        case Setting_NetMask:
            ip4addr_ntoa_r((const ip_addr_t *)&wifi.sta.network.mask, ip, IPADDR_STRLEN_MAX);
            break;

#if WIFI_SOFTAP

        case Setting_IpAddress2:
            ip4addr_ntoa_r((const ip_addr_t *)&wifi.ap.network.ip, ip, IPADDR_STRLEN_MAX);
            break;

        case Setting_Gateway2:
            ip4addr_ntoa_r((const ip_addr_t *)&wifi.ap.network.gateway, ip, IPADDR_STRLEN_MAX);
            break;

        case Setting_NetMask2:
            ip4addr_ntoa_r((const ip_addr_t *)&wifi.ap.network.mask, ip, IPADDR_STRLEN_MAX);
            break;

#endif

        default:
            *ip = '\0';
            break;
    }

    return ip;
}

static void wifi_settings_restore (void)
{
    ip4_addr_t addr;

    memset(&wifi, 0, sizeof(wifi_settings_t));

    wifi.mode = WiFiMode_STA;

// Station

    strlcpy(wifi.sta.network.hostname, NETWORK_HOSTNAME, sizeof(wifi.sta.network.hostname));

    wifi.sta.network.ip_mode = (ip_mode_t)NETWORK_IPMODE;

    if(ip4addr_aton(NETWORK_IP, &addr) == 1)
        set_addr(wifi.sta.network.ip, &addr);

    if(ip4addr_aton(NETWORK_GATEWAY, &addr) == 1)
        set_addr(wifi.sta.network.gateway, &addr);

#if NETWORK_IPMODE == 0
    if(ip4addr_aton(NETWORK_MASK, &addr) == 1)
        set_addr(wifi.sta.network.mask, &addr);
 #else
    if(ip4addr_aton("255.255.255.0", &addr) == 1)
        set_addr(wifi.sta.network.mask, &addr);
#endif

// Access Point

#if WIFI_SOFTAP

    wifi.ap.network.ip_mode = IpMode_Static;
    strlcpy(wifi.ap.network.hostname, NETWORK_AP_HOSTNAME, sizeof(wifi.ap.network.hostname));
    strlcpy(wifi.ap.ssid, NETWORK_AP_SSID, sizeof(wifi.ap.ssid));
    strlcpy(wifi.ap.password, NETWORK_AP_PASSWORD, sizeof(wifi.ap.password));

    if(ip4addr_aton(NETWORK_AP_IP, &addr) == 1)
        set_addr(wifi.ap.network.ip, &addr);

    if(ip4addr_aton(NETWORK_AP_GATEWAY, &addr) == 1)
        set_addr(wifi.ap.network.gateway, &addr);

    if(ip4addr_aton(NETWORK_AP_MASK, &addr) == 1)
        set_addr(wifi.ap.network.mask, &addr);

#endif

// Common

    wifi.sta.network.telnet_port = wifi.ap.network.telnet_port = NETWORK_TELNET_PORT;
    wifi.sta.network.ftp_port = wifi.ap.network.ftp_port = NETWORK_FTP_PORT;
    wifi.sta.network.http_port = wifi.ap.network.http_port = NETWORK_HTTP_PORT;
    wifi.sta.network.websocket_port = wifi.ap.network.websocket_port = NETWORK_WEBSOCKET_PORT;
    wifi.sta.network.services = wifi.ap.network.services = allowed_services;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&wifi, sizeof(wifi_settings_t), true);
}

static void wifi_settings_load (void)
{
    ip4_addr_t addr;

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&wifi, nvs_address, sizeof(wifi_settings_t), true) != NVS_TransferResult_OK)
        wifi_settings_restore();

// Sanity checks

    // AP ip/gateway adresses are hardcoded!
    if(ip4addr_aton("192.168.4.1", &addr) == 1) {
        set_addr(wifi.ap.network.ip, &addr);
        set_addr(wifi.ap.network.gateway, &addr);
    }

#if WIFI_SOFTAP
    if(wifi.mode == WiFiMode_APSTA)
        wifi.mode = WiFiMode_STA;
#else
    if(wifi.mode == WiFiMode_AP || wifi.mode == WiFiMode_APSTA)
        wifi.mode = WiFiMode_STA;
#endif

    wifi.sta.network.services.mask &= allowed_services.mask;
    wifi.ap.network.services.mask &= allowed_services.mask;
}

static void stream_changed (stream_type_t type)
{
    if(type != StreamType_SDCard)
        active_stream = type;

    if(on_stream_changed)
        on_stream_changed(type);
}

extern network_services_t networking_get_services_list (char *list);

bool wifi_init (void)
{
    if((nvs_address = nvs_alloc(sizeof(wifi_settings_t)))) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = reportIP;

        on_stream_changed = grbl.on_stream_changed;
        grbl.on_stream_changed = stream_changed;

        settings_register(&setting_details);

        allowed_services.mask = networking_get_services_list((char *)netservices).mask;
    }

    return nvs_address != 0;
}

#endif // WIFI_ENABLE
