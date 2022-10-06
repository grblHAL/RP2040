/*
  my_machine.h - configuration for Raspberry RP2040 ARM processors

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

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used.
// #define BOARD_PICO_CNC
//#define BOARD_PICOBOB
//#define BOARD_BTT_SKR_PICO_10 // incomplete and untested!
//#define BOARD_CNC_BOOSTERPACK
//#define BOARD_CITOH_CX6000    // C.ITOH CX-6000 HPGL plotter
//#define BOARD_MY_MACHINE      // Add my_machine_map.h before enabling this!
#define BOARD_MRBEAM_PICOGRBL 

// Configuration
// Uncomment to enable.

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC       1 // Serial communication via native USB.
#endif
//#define SAFETY_DOOR_ENABLE   1 // Enable safety door input.
//#define BLUETOOTH_ENABLE     1 // Set to 1 for HC-05 module.
//#define VFD_ENABLE           1 // Set to 1 or 2 for Huanyang VFD spindle. More here https://github.com/grblHAL/Plugins_spindle
//#define MODBUS_ENABLE        1 // Set to 1 for auto direction, 2 for direction signal on auxillary output pin.
//#define WIFI_ENABLE          0 // Do NOT enable here, enable in CMakeLists.txt!
//#define WIFI_SOFTAP          1 // Use Soft AP mode for WiFi. NOTE: WIP - not yet complete!
//#define WEBUI_ENABLE         1 // Enable ESP3D-WEBUI plugin along with networking and SD card plugins. Requires WiFi enabled.
//#define WEBUI_AUTH_ENABLE    1 // Enable ESP3D-WEBUI authentication.
//#define WEBUI_INFLASH        1 // Store WebUI files in flash instead of on SD card.
//#define SDCARD_ENABLE        1 // Run gcode programs from SD card.
//#define MPG_ENABLE           2 // Enable MPG interface. Requires serial port and one handshake pin unless
                                 // KEYPAD_ENABLE is set to 2 when mode switching is done by the CMD_MPG_MODE_TOGGLE (0x8B)
                                 // command character. Set both MPG_ENABLE and KEYPAD_ENABLE to 2 to use a handshake pin anyway.
//#define KEYPAD_ENABLE        2 // Set to 1 for I2C keypad, 2 for other input such as serial data. If KEYPAD_ENABLE is set to 2 
                                 // and MPG_ENABLE is uncommented then the serial stream is shared with the MPG.
//#define ODOMETER_ENABLE      1 // Odometer plugin.
//#define PPI_ENABLE           1 // Laser PPI plugin. To be completed.
//#define LASER_COOLANT_ENABLE 1 // Laser coolant plugin. To be completed.
//#define TRINAMIC_ENABLE      1 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_I2C         1 // Trinamic I2C - SPI bridge interface.
//#define TRINAMIC_DEV         1 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code.
//#define EEPROM_ENABLE        1 // I2C EEPROM support. Set to 1 for 24LC16(2K), 2 for larger sizes.
//#define EEPROM_IS_FRAM       1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.

// If the selected board map supports more than three motors ganging and/or auto-squaring
// of axes can be enabled here.
//#define X_GANGED             1
//#define X_AUTO_SQUARE        1
//#define Y_GANGED             1
//#define Y_AUTO_SQUARE        1
//#define Z_GANGED             1
//#define Z_AUTO_SQUARE        1
// For ganged axes the limit switch input (if available) can be configured to act as a max travel limit switch.
// NOTE: If board map already has max limit inputs defined this configuration will be ignored.
//#define X_GANGED_LIM_MAX     1
//#define Y_GANGED_LIM_MAX     1
//#define Z_GANGED_LIM_MAX     1
//

#if WIFI_ENABLE > 0
#define TELNET_ENABLE        1 // Telnet daemon - requires WiFi streaming enabled.
#define WEBSOCKET_ENABLE     1 // Websocket daemon - requires WiFi streaming enabled.
#ifdef SDCARD_ENABLE
//#define FTP_ENABLE           1 // Ftp daemon - requires SD card enabled.
//#define HTTP_ENABLE          1 // http daemon - requires SD card enabled.
//#define WEBDAV_ENABLE        1 // webdav protocol - requires http daemon and SD card enabled.
#endif
// The following symbols have the default values as shown, uncomment and change as needed.
//#define NETWORK_HOSTNAME     "grblHAL"
//#define NETWORK_IPMODE       1 // 0 = static, 1 = DHCP, 2 = AutoIP
//#define NETWORK_IP           "192.168.5.1"
//#define NETWORK_GATEWAY      "192.168.5.1"
//#define NETWORK_MASK         "255.255.255.0"
//#define NETWORK_FTP_PORT     21
//#define NETWORK_TELNET_PORT  23
//#define NETWORK_HTTP_PORT    80
#if WIFI_SOFTAP > 0
//#define NETWORK_AP_SSID      "grblHAL_AP"
//#define NETWORK_AP_PASSWORD  "grblHAL"
//#define NETWORK_AP_HOSTNAME  "grblHAL_AP"
//#define NETWORK_AP_IPMODE    0 // 0 = static, 1 = DHCP, 2 = AutoIP
//#define NETWORK_AP_IP        "192.168.5.1"
//#define NETWORK_AP_GATEWAY   "192.168.5.1"
//#define NETWORK_AP_MASK      "255.255.255.0"
#endif
#if HTTP_ENABLE
//#define NETWORK_WEBSOCKET_PORT  81
#else
//#define NETWORK_WEBSOCKET_PORT  80
#endif
#endif

/**/
