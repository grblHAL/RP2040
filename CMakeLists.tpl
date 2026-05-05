cmake_minimum_required(VERSION 3.13)

%compile_options%

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
include(trinamic/CMakeLists.txt)
include(spindle/CMakeLists.txt)
include(keypad/CMakeLists.txt)
include(embroidery/CMakeLists.txt)
include(laser/CMakeLists.txt)
include(plasma/CMakeLists.txt)
include(plugins/CMakeLists.txt)
%include_libraries%

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
    sr16_out.c
    ioports.c
    ioports_analog.c
    tmc_uart.c
    my_plugin.c
    boards/pico_cnc.c
    boards/btt_skr_pico_10.c
    eeprom/eeprom_24AAxxx.c
    eeprom/eeprom_24LC16B.c
    keypad/keypad.c
    sdcard/sdcard.c
    sdcard/ymodem.c
    sdcard/fs_fatfs.c
    sdcard/fs_littlefs.c
    sdcard/fs_stream.c
    sdcard/macros.c
    bluetooth/hc_05.c
    motors/trinamic.c
    littlefs/lfs.c
    littlefs/lfs_util.c
    littlefs_hal.c
    fans/fans.c
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
    sr16_out.c
    ioports.c
    ioports_analog.c
    tmc_uart.c
    boards/pico_cnc.c
    boards/btt_skr_pico_10.c
    eeprom/eeprom_24AAxxx.c
    eeprom/eeprom_24LC16B.c
    keypad/keypad.c
    sdcard/sdcard.c
    sdcard/ymodem.c
    sdcard/fs_fatfs.c
    sdcard/fs_littlefs.c
    sdcard/fs_stream.c
    sdcard/macros.c
    bluetooth/hc_05.c
    motors/trinamic.c
    littlefs/lfs.c
    littlefs/lfs_util.c
    littlefs_hal.c
    fans/fans.c
  )
endif()

pico_generate_pio_header(grblHAL ${CMAKE_CURRENT_LIST_DIR}/driverPIO.pio)

if(PICO_SDK_VERSION_STRING VERSION_GREATER_EQUAL "2.1.1")
target_compile_definitions(grblHAL PUBLIC PICO_USE_FASTEST_SUPPORTED_CLOCK=1)
endif()

target_compile_definitions(grblHAL PUBLIC RP2040)
target_compile_definitions(grblHAL PUBLIC NEW_FATFS)
target_compile_definitions(grblHAL PUBLIC LITTLEFS_ENABLE=1)
%compile_definitions%

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
     bluetooth.c
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

target_include_directories(grblHAL PRIVATE ${CMAKE_CURRENT_LIST_DIR})

if(PICO_PLATFORM STREQUAL "rp2040")
    target_link_libraries(grblHAL PRIVATE
        hardware_rtc
    )
endif()

target_link_libraries(grblHAL PRIVATE
 grbl
 fatfs
 trinamic
 spindle
 keypad
 laser
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
%link_libraries%
)

pico_add_extra_outputs(grblHAL)

unset(PICO_BOARD CACHE)
unset(PICO_PLATFORM CACHE)
unset(ADD_WIFI CACHE)
unset(ADD_ETHERNET CACHE)
unset(ADD_BLUETOOTH CACHE)
unset(ADD_HPGL CACHE)
unset(ADD_mDNS CACHE)
unset(AddMyPlugin CACHE)
