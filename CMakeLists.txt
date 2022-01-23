cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)
include(grbl/CMakeLists.txt)
include(fatfs/CMakeLists.txt)
include(trinamic/CMakeLists.txt)
# include(hpgl/CMakeLists.txt)

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
 bluetooth/hc_05.c
 motors/trinamic.c
 citoh_cx6000.c
)

pico_generate_pio_header(grblHAL ${CMAKE_CURRENT_LIST_DIR}/driverPIO.pio)

target_compile_definitions(grblHAL PUBLIC RP2040)
target_compile_definitions(grblHAL PUBLIC NEW_FATFS)
target_include_directories(grblHAL PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(grblHAL PRIVATE
 grbl
# hpgl
 fatfs
 trinamic
 tinyusb_device_unmarked
 pico_stdlib
 hardware_uart
 hardware_pio
 hardware_i2c
 hardware_spi
 hardware_gpio
 hardware_pwm
 hardware_clocks
 hardware_flash
)

# create map/bin/hex file etc.
pico_add_extra_outputs(grblHAL)
