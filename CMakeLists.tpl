cmake_minimum_required(VERSION 3.13)

%compile_options%

if(ADD_WIFI)
set(PICO_BOARD pico_w)
endif()

include(pico_sdk_import.cmake)
if(ADD_WIFI)
include(networking/CMakeLists.txt)
include(webui/CMakeLists.txt)
if(ADD_mDNS OR ADD_MQTT)
set(LWIP_DIR /home/sdk/pico-sdk/lib/lwip) # LWIP_DIR
include(${LWIP_DIR}/src/Filelists.cmake)
endif()
endif()
include(grbl/CMakeLists.txt)
include(fatfs/CMakeLists.txt)
include(trinamic/CMakeLists.txt)
include(spindle/CMakeLists.txt)
if(ADD_HPGL)
include(hpgl/CMakeLists.txt)
endif()

project(grblHAL)
pico_sdk_init()

add_executable(grblHAL
 main.c
 driver.c
 serial.c
 usb_serial.c
 stdio_usb_descriptors.c
 flash.c
 i2c.c
 PCA9654E.c
 pico_cnc.c
 btt_skr_pico_10.c
 ioports.c
 tmc_uart.c
 eeprom/eeprom_24AAxxx.c
 eeprom/eeprom_24LC16B.c
 keypad/keypad.c
 sdcard/sdcard.c
 sdcard/ymodem.c
 sdcard/fs_fatfs.c
 sdcard/fs_littlefs.c
 bluetooth/hc_05.c
 motors/trinamic.c
 littlefs/lfs.c
 littlefs/lfs_util.c
 littlefs_hal.c
)

pico_generate_pio_header(grblHAL ${CMAKE_CURRENT_LIST_DIR}/driverPIO.pio)

target_compile_definitions(grblHAL PUBLIC RP2040)
target_compile_definitions(grblHAL PUBLIC NEW_FATFS)
target_compile_definitions(grblHAL PUBLIC LITTLEFS_ENABLE=1)
%compile_definitions%

if(ADD_WIFI)
target_compile_definitions(grblHAL PUBLIC WIFI_ENABLE=1)
target_compile_definitions(grblHAL PUBLIC NDEBUG=0)
target_compile_definitions(grblHAL PUBLIC WEBUI_AUTO_REPORT_INTERVAL=0)
target_sources(grblHAL PRIVATE
 wifi.c
 dhcpserver.c
)
target_link_libraries(grblHAL PRIVATE
 networking
 webui
 pico_cyw43_arch_lwip_poll
)
if(ADD_mDNS)
target_compile_definitions(grblHAL PUBLIC LWIP_IGMP=1)
target_compile_definitions(grblHAL PUBLIC LWIP_NUM_NETIF_CLIENT_DATA=1)
target_compile_definitions(grblHAL PUBLIC LWIP_MDNS_RESPONDER=1)
target_compile_definitions(grblHAL PUBLIC MDNS_MAX_SERVICES=8)
target_compile_definitions(grblHAL PUBLIC MDNS_ENABLE=1)
target_sources(grblHAL PRIVATE
 ${lwipmdns_SRCS}
)
endif()
if(ADD_MQTT)
target_compile_definitions(grblHAL PUBLIC MQTT_ENABLE=1)
target_sources(grblHAL PRIVATE
 ${lwipmqtt_SRCS}
)
endif()
endif()

if(ADD_HPGL)
target_sources(grblHAL PRIVATE
 citoh_cx6000.c
)
target_link_libraries(grblHAL PRIVATE
 hpgl
)
endif()

target_include_directories(grblHAL PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(grblHAL PRIVATE
 grbl
 fatfs
 trinamic
 spindle
 tinyusb_device_unmarked
 pico_stdlib
 hardware_uart
 hardware_pio
 hardware_i2c
 hardware_spi
 hardware_gpio
 hardware_pwm
 hardware_rtc
 hardware_clocks
 hardware_flash
)

pico_add_extra_outputs(grblHAL)

unset(ADD_WIFI CACHE)
unset(ADD_HPGL CACHE)
unset(ADD_mDNS CACHE)
