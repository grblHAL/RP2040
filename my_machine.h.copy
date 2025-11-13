/*
  my_machine.h - configuration for Raspberry RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

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

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used.
//#define BOARD_PICO_CNC
//#define BOARD_RP23U5XBB
//#define BOARD_PICOBOB
//#define BOARD_PICOBOB_G540
//#define BOARD_PICOBOB_DLX
//#define BOARD_PICOBOB_DLX_G540
//#define BOARD_PICOHAL
//#define BOARD_BTT_SKR_PICO_10 // incomplete and untested!
//#define BOARD_CNC_BOOSTERPACK
//#define BOARD_CITOH_CX6000    // C.ITOH CX-6000 HPGL plotter
//#define BOARD_GENERIC_4AXIS
//#define BOARD_GENERIC_8AXIS
//#define BOARD_MY_MACHINE      // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable.

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC          1 // Serial communication via native USB.
#endif
//#define BLUETOOTH_ENABLE        2 // Set to 2 for HC-05 module, enable in CMakeLists.txt if for Pico W Bluetooth.
// Spindle selection:
// Up to four specific spindle drivers can be instantiated at a time
// depending on N_SPINDLE and N_SYS_SPINDLE definitions in grbl/config.h.
// If none are specified the default PWM spindle is instantiated.
// Spindle definitions can be found in grbl/spindle_control.h.
// More here https://github.com/grblHAL/Plugins_spindle
//#define SPINDLE0_ENABLE         SPINDLE_HUANYANG1
//#define SPINDLE1_ENABLE         SPINDLE_PWM0
//#define SPINDLE2_ENABLE         SPINDLE_NONE 
//#define SPINDLE2_ENABLE         SPINDLE_NONE
//#define SPINDLE_OFFSET          1 // Set to 1 to add offset move when switching between laser and spindle
// **********************
//#define MODBUS_ENABLE           1 // Set to 1 for auto direction, 2 for direction signal on auxiliary output pin.
//#define WIFI_ENABLE             0 // Do NOT enable here, enable in CMakeLists.txt!
//#define WIFI_SOFTAP             1 // Use Soft AP mode for WiFi. NOTE: WIP - not yet complete!
//#define ETHERNET_ENABLE         0 // Do NOT enable here, enable in CMakeLists.txt!
//#define _WIZCHIP_            5500 // Selects WIZnet ethernet breakout connected via SPI.
                                    // Uncomment to enable W5500 chip, default is W5100S. Requires ethernet enabled in CMakeLists.txt.
//#define WEBUI_ENABLE            3 // Enable ESP3D-WEBUI plugin along with networking and SD card plugins. Requires WiFi enabled.
//#define WEBUI_AUTH_ENABLE       1 // Enable ESP3D-WEBUI authentication.
//#define WEBUI_INFLASH           0 // Uncomment to store WebUI files on SD card instead of in flash (littlefs).
//#define SDCARD_ENABLE           2 // Run gcode programs from SD card. Set to 2 to enable YModem upload.
//#define MPG_ENABLE              2 // Enable MPG interface. Requires a serial stream and means to switch between normal and MPG mode.
                                    // 1: Mode switching is by handshake pin.
                                    // 2: Mode switching is by the CMD_MPG_MODE_TOGGLE (0x8B) command character.
//#define KEYPAD_ENABLE           1 // 1: uses a I2C keypad for input.
                                    // 2: uses a serial stream for input. If MPG_ENABLE is set > 0 the serial stream is shared with the MPG.
//#define DISPLAY_ENABLE          9 // Set to 9 for I2C display protocol, 17 for I2C LED protocol.
//#define ODOMETER_ENABLE         1 // Odometer plugin.
//#define PLASMA_ENABLE           1 // Plasma (THC) plugin. To be completed.
//#define LASER_COOLANT_ENABLE    1 // Laser coolant plugin. To be completed.
//#define LASER_OVD_ENABLE        1 // Enable M-code for overdrive PWM output during spindle off in RPM controlled mode.
//#define LB_CLUSTERS_ENABLE      1 // LaserBurn cluster support.
//#define FANS_ENABLE             1 // Enable fan control via M106/M107. Activates fan plugin.
//#define EMBROIDERY_ENABLE       1 // Embroidery plugin. To be completed.
//#define TRINAMIC_ENABLE         1 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_I2C            1 // Trinamic I2C - SPI bridge interface.
//#define TRINAMIC_DEV            1 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code.
//#define EEPROM_ENABLE          16 // I2C EEPROM/FRAM support. Set to 16 for 2K, 32 for 4K, 64 for 8K, 128 for 16K and 256 for 16K capacity.
//#define EEPROM_IS_FRAM          1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.
//#define STEP_INJECT_ENABLE      1
//#define RGB_LED_ENABLE          2 // Set to 1 to enable strip length settings $536 and $537, set to 2 to also enable M150 LED strip control.
//#define PWM_SERVO_ENABLE        1 // Enable M280 PWM servo support, requires at least one PWM capable auxiliary output.
//#define BLTOUCH_ENABLE          1 // Enable M401/M402 BLTouch support. Requires and claims one auxiliary PWM servo output.
//#define EVENTOUT_ENABLE         1 // Enable binding events (triggers) to control auxiliary outputs.
//#define ESP_AT_ENABLE           1 // Enable support for Telnet communication via UART connected ESP32 running ESP-AT.
//#define FEED_OVERRIDE_ENABLE    1 // Enable M200 feed override control.
//#define HOMING_PULLOFF_ENABLE   1 // Enable per axis homing pulloff distance settings.

// IO expanders:
//
//#define MCP3221_ENABLE          1 // MCP3221 I2C ADC input, default address is 0x9A (MCP3221_ADDRESS).
//#define MCP4725_ENABLE          1 // MCP3221 I2C ADC input, default address is 0xC0 (MCP3221_ADDRESS).
//#define PCA9654E_ENABLE         1 // PCA9654E I2C digital I/O, default address is 0x40 (PCA9654E_ADDRESS).
//#define THCAD2_ENABLE           1 // Mesa THCAD2 analog to frequency converter. Not yet completed!

// Optional control signals:
// These will be assigned to aux input pins. Use the $pins command to check which pins are assigned.
// NOTE: If not enough pins are available assignment will silently fail.
//#define PROBE_ENABLE            0 // Default enabled, remove comment to disable probe input.
//#define PROBE2_ENABLE           1 // Enable second regular probe input, depending on the board the input assigned may be predefined.
//#define TOOLSETTER_ENABLE       1 // Enable toolsetter input, depending on the board the input assigned may be predefined.
//#define SAFETY_DOOR_ENABLE      1
//#define MOTOR_FAULT_ENABLE      1
//#define MOTOR_WARNING_ENABLE    1
//#define PROBE_DISCONNECT_ENABLE 1
//#define STOP_DISABLE_ENABLE     1
//#define BLOCK_DELETE_ENABLE     1
//#define SINGLE_BLOCK_ENABLE     1
//#define LIMITS_OVERRIDE_ENABLE  1

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

#if WIFI_ENABLE || ETHERNET_ENABLE || WEBUI_ENABLE
#define TELNET_ENABLE        1 // Telnet daemon - requires WiFi streaming enabled.
#define WEBSOCKET_ENABLE     1 // Websocket daemon - requires WiFi streaming enabled.
//#define MDNS_ENABLE          0 // mDNS daemon. Do NOT enable here, enable in CMakeLists.txt!
//#define SSDP_ENABLE          1 // SSDP daemon - requires HTTP enabled.
#if SDCARD_ENABLE || WEBUI_ENABLE
#define FTP_ENABLE           1 // Ftp daemon - requires SD card enabled.
//#define HTTP_ENABLE          1 // http daemon - requires SD card enabled.
//#define WEBDAV_ENABLE        1 // webdav protocol - requires http daemon and SD card enabled.
#endif
// The following symbols have the default values as shown, uncomment and change as needed.
//#define NETWORK_STA_HOSTNAME    "grblHAL"
//#define NETWORK_STA_IPMODE      1 // 0 = static, 1 = DHCP, 2 = AutoIP
//#define NETWORK_STA_IP          "192.168.5.1"
//#define NETWORK_STA_GATEWAY     "192.168.5.1"
//#define NETWORK_STA_MASK        "255.255.255.0"
#if WIFI_SOFTAP > 0
//#define NETWORK_AP_SSID         "grblHAL_AP"
//#define NETWORK_AP_PASSWORD     "grblHALap"
//#define NETWORK_AP_HOSTNAME     "grblHAL_AP"
//#define NETWORK_AP_IPMODE       0              // Do not change!
//#define NETWORK_AP_IP           "192.168.4.1"  // Do not change!
//#define NETWORK_AP_GATEWAY      "192.168.4.1"  // Do not change!
//#define NETWORK_AP_MASK         "255.255.255.0"
#endif
//#define NETWORK_FTP_PORT     21
//#define NETWORK_TELNET_PORT  23
//#define NETWORK_HTTP_PORT    80
#if HTTP_ENABLE
//#define NETWORK_WEBSOCKET_PORT  81
#else
//#define NETWORK_WEBSOCKET_PORT  80
#endif
#endif // WIFI_ENABLE

/**/
