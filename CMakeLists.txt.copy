# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

option(ADD_HPGL "Add HPGL plugin" OFF)
 # Note: WiFi and Ethernet cannot be enabled at the same time!
option(ADD_WIFI "Add WiFi networking" OFF)
option(ADD_ETHERNET "Add Ethernet networking" OFF)
option(ADD_BLUETOOTH "Add Bluetooth" OFF)
# Set LWIP_DIR below to the root path of lwIP sources when the following is set ON!
option(ADD_mDNS "Add mDNS service" OFF) 
option(ADD_MQTT "Add MQTT client API" OFF)
#
option(AddMyPlugin "Add my_plugin.c to build" OFF)

# Select the correct board in VSCode, the following line will be updated accordingly
set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

if(ADD_WIFI OR ADD_ETHERNET)
    include(networking/CMakeLists.txt)
    include(webui/CMakeLists.txt)
    if(ADD_mDNS OR ADD_MQTT)
        set(LWIP_DIR ${PICO_SDK_PATH}/lib/lwip)
        include(${LWIP_DIR}/src/Filelists.cmake)
    endif()
endif()

include(grbl/CMakeLists.txt)
include(fatfs/CMakeLists.txt)
include(motors/CMakeLists.txt)
include(trinamic/CMakeLists.txt)
include(spindle/CMakeLists.txt)
include(sdcard/CMakeLists.txt)
include(keypad/CMakeLists.txt)
include(eeprom/CMakeLists.txt)
include(fans/CMakeLists.txt)
include(bluetooth/CMakeLists.txt)
include(embroidery/CMakeLists.txt)
include(plasma/CMakeLists.txt)
include(laser/CMakeLists.txt)
include(plugins/CMakeLists.txt)
if(ADD_HPGL)
    include(hpgl/CMakeLists.txt)
endif()

project(grblHAL)
pico_sdk_init()

if(AddMyPlugin)
    add_executable(grblHAL
    main.c
    driver.c
    serial.c
    usb_serial.c
    stdio_usb_descriptors.c
    flash.c
    i2c.c
    spi.c
    thcad2.c
    sr16_out.c
    ioports.c
    ioports_analog.c
    tmc_uart.c
    my_plugin.c
    boards/pico_cnc.c
    boards/btt_skr_pico_10.c
    littlefs/lfs.c
    littlefs/lfs_util.c
    littlefs_hal.c
    )
else()
    add_executable(grblHAL
    main.c
    driver.c
    serial.c
    usb_serial.c
    stdio_usb_descriptors.c
    flash.c
    i2c.c
    spi.c
    thcad2.c
    sr16_out.c
    ioports.c
    ioports_analog.c
    tmc_uart.c
    boards/pico_cnc.c
    boards/btt_skr_pico_10.c
    littlefs/lfs.c
    littlefs/lfs_util.c
    littlefs_hal.c
    )
endif()

pico_generate_pio_header(grblHAL ${CMAKE_CURRENT_LIST_DIR}/driverPIO.pio)

if(PICO_BOARD STREQUAL "pico" OR PICO_BOARD STREQUAL "pico_w")
target_compile_definitions(grblHAL PUBLIC RP_MCU=2040)
if(PICO_SDK_VERSION_STRING VERSION_GREATER_EQUAL "2.1.1")
target_compile_definitions(grblHAL PUBLIC PICO_USE_FASTEST_SUPPORTED_CLOCK=1)
endif()
else()
target_compile_definitions(grblHAL PUBLIC RP_MCU=2350)
endif()
target_compile_definitions(grblHAL PUBLIC RP2040)
target_compile_definitions(grblHAL PUBLIC NEW_FATFS)
#target_compile_definitions(grblHAL PUBLIC LITTLEFS_ENABLE=1)

if(ADD_WIFI)
    target_compile_definitions(grblHAL PUBLIC WIFI_ENABLE=1)
    target_compile_definitions(grblHAL PUBLIC NDEBUG)
    target_sources(grblHAL PRIVATE
     wifi.c
     dhcpserver.c
    )
    target_link_libraries(grblHAL PRIVATE
     pico_cyw43_arch_lwip_poll
    )
endif()

if(ADD_ETHERNET)
    target_compile_definitions(grblHAL PUBLIC ETHERNET_ENABLE=1)
    target_compile_definitions(grblHAL PUBLIC CYW43_LWIP=0)
    target_compile_definitions(grblHAL PUBLIC LWIP_NOASSERT)
    include(networking/wiznet/CMakeLists.txt)
    target_sources(grblHAL PRIVATE
     w5x00_ll_driver.c
    )
    target_link_libraries(grblHAL PRIVATE
     wiznet
     pico_lwip
     pico_lwip_nosys
    )
endif()

if(ADD_WIFI OR ADD_ETHERNET)
    target_compile_definitions(grblHAL PUBLIC WEBUI_AUTO_REPORT_INTERVAL=0)
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
        target_link_libraries(grblHAL PRIVATE
            pico_lwip_mqtt
        )
    endif()
    target_link_libraries(grblHAL PRIVATE
     networking
     webui
    )
endif()

if(ADD_BLUETOOTH)
    target_compile_definitions(grblHAL PUBLIC BLUETOOTH_ENABLE=1)
    target_compile_definitions(grblHAL PUBLIC NDEBUG)
    target_sources(grblHAL PRIVATE
     bt_native.c
    )
    target_include_directories(grblHAL PRIVATE ${PICO_SDK_PATH}/lib/btstack/src)
    target_link_libraries(grblHAL PRIVATE
     pico_btstack_classic 
     pico_btstack_ble
     pico_btstack_cyw43
    )
    if(NOT ADD_WIFI)
        target_compile_definitions(grblHAL PUBLIC CYW43_LWIP=0)
        target_link_libraries(grblHAL PRIVATE
         pico_cyw43_arch_poll
        )
    endif()
endif()

if(AddMyPlugin)
    target_sources(grblHAL PRIVATE
     my_plugin.c
    )
endif()

if(ADD_HPGL)
    target_sources(grblHAL PRIVATE
     boards/citoh_cx6000.c
    )
    target_link_libraries(grblHAL PRIVATE
     hpgl
    )
endif()

target_include_directories(grblHAL PRIVATE ${CMAKE_CURRENT_LIST_DIR})
if(PICO_PLATFORM STREQUAL "rp2040" OR PICO_BOARD STREQUAL "pico" OR PICO_BOARD STREQUAL "pico_w")
target_compile_definitions(grblHAL PUBLIC RP_MCU=2040)
target_link_libraries(grblHAL PRIVATE
 hardware_rtc
)
else()
target_compile_definitions(grblHAL PUBLIC RP_MCU=2350)
endif()

target_link_libraries(grblHAL PRIVATE
 grbl
 fatfs
 sdcard
 motors
 trinamic
 spindle
 keypad
 fans
 laser
 bluetooth
 eeprom
 plasma
 misc_plugins
 embroidery
 tinyusb_device_unmarked
 pico_stdlib
 pico_unique_id
 hardware_dma
 hardware_uart
 hardware_pio
 hardware_i2c
 hardware_spi
 hardware_gpio
 hardware_pwm
 hardware_clocks
 hardware_flash
)

pico_add_extra_outputs(grblHAL)

unset(PICO_BOARD CACHE)
unset(ADD_WIFI CACHE)
unset(ADD_ETHERNET CACHE)
unset(ADD_BLUETOOTH CACHE)
unset(ADD_HPGL CACHE)
unset(ADD_mDNS CACHE)
unset(AddMyPlugin CACHE)
