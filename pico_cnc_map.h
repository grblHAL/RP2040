/*
  pico_cnc_map.h - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021-2023 Terje Io

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

#include <stdint.h>

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "PicoCNC"
#define BOARD_URL "https://github.com/phil-barrett/PicoCNC"
#define HAS_BOARD_INIT

typedef union {
    uint32_t value;
    struct {
        uint32_t spi_reset   :1,
                 aux6_out    :1,
                 aux5_out    :1,
                 aux4_out    :1,
                 aux3_out    :1,
                 aux2_out    :1,
                 aux1_out    :1,
                 aux0_out    :1,
                 flood_ena   :1,
                 mist_ena    :1,
                 spindle_ena :1,
                 spindle_dir :1,
                 m3_ena      :1,
                 z_ena       :1,
                 y_ena       :1,
                 x_ena       :1,
                 unused      :16;
    };
} output_sr_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t m3_dir  :1,
                z_dir   :1,
                y_dir   :1,
                x_dir   :1,
                m3_step :1,
                z_step  :1,
                y_step  :1,
                x_step  :1;
    };
} step_dir_t;

typedef union {
    uint32_t value;
    struct {
        step_dir_t set;
        step_dir_t reset;
        uint16_t unused;
    };
} step_dir_sr_t;

// Define step pulse output pins.
#define SD_SHIFT_REGISTER   8
#define SD_SR_DATA_PIN      14
#define SD_SR_SCK_PIN       15 // includes next pin (16)

// Define output signals pins.
#define OUT_SHIFT_REGISTER  16
#define OUT_SR_DATA_PIN     17
#define OUT_SR_SCK_PIN      18 // includes next pin (19)

#define STEP_PORT           GPIO_SR8
#define DIRECTION_PORT      GPIO_SR8
#define ENABLE_PORT         GPIO_SR16

#define AUX_N_OUT           8
#define AUX_OUT_MASK        0xFF

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN         0 // Not referenced by driver code
#define M3_DIRECTION_PIN    0 // Not referenced by driver code
#define M3_LIMIT_PIN        3
#define M3_ENABLE_PIN       0 // Not referenced by driver code
#endif

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN         6
#define Y_LIMIT_PIN         5
#define Z_LIMIT_PIN         4

#define SPINDLE_PORT        GPIO_SR16
#define COOLANT_PORT        GPIO_SR16

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT    GPIO_OUTPUT
#define SPINDLE_PWM_PIN     27

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PIN           22
#define FEED_HOLD_PIN       7
#define CYCLE_START_PIN     8
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN     9
#endif

// Define probe switch input pin.
#define PROBE_PIN           28

#define AUX_IO0_PIN         10
#define AUX_IO1_PIN         11
#define AUX_IO2_PIN         12
#define AUX_IO3_PIN         13
#define AUX_IO4_PIN         2 // Modbus direction or ethernet CS

#if !(SDCARD_ENABLE || ETHERNET_ENABLE) || !defined(SAFETY_DOOR_PIN)
#if !(SDCARD_ENABLE || ETHERNET_ENABLE) 
#define AUXINPUT0_PIN       AUX_IO0_PIN
#define AUXINPUT1_PIN       AUX_IO1_PIN
#define AUXINPUT2_PIN       AUX_IO2_PIN
#if MPG_MODE != 1
//#define AUXINPUT3_PIN       AUX_IO3_PIN
#define AUXOUTPUT0_PWM_PIN AUX_IO3_PIN
#endif
#ifndef SAFETY_DOOR_PIN
#define AUXINPUT4_PIN       9   
#endif
#else
#define AUXINPUT0_PIN       9   
#endif
#endif

#define AUXOUTPUT0_PWM_PIN AUX_IO3_PIN

#if I2C_ENABLE
#define I2C_PORT            0
#define I2C_SDA             20
#define I2C_SCL             21
#endif

#if SDCARD_ENABLE
#define SD_CS_PIN           AUX_IO3_PIN
#endif

#if ETHERNET_ENABLE
  #if !SDCARD_ENABLE
    #define SPI_CS_PIN      AUX_IO3_PIN
    #define AUXOUTPUT7_PORT GPIO_OUTPUT
    #define AUXOUTPUT7_PIN  AUX_IO4_PIN
  #else
    #define SPI_CS_PIN      AUX_IO4_PIN
  #endif
  #define SPI_IRQ_PIN       26
  #define SPI_RST_PORT      GPIO_SR16
#else
  #define AUXOUTPUT7_PORT   GPIO_OUTPUT
  #define AUXOUTPUT7_PIN    AUX_IO4_PIN
#endif

#if SDCARD_ENABLE || ETHERNET_ENABLE
#define SPI_PORT            1
#define SPI_SCK_PIN         AUX_IO0_PIN
#define SPI_MOSI_PIN        AUX_IO1_PIN
#define SPI_MISO_PIN        AUX_IO2_PIN
#endif

#if MPG_MODE == 1
    #if defined(SPI_CS_PIN) && SPI_CS_PIN == AUX_IO3_PIN
      #error "MPG mode select pin not available, is assigned as SPI CS!"
    #else
      #define MPG_MODE_PIN AUX_IO3_PIN
    #endif
#endif

#if MODBUS_ENABLE & MODBUS_RTU_DIR_ENABLED
  #if defined(SPI_CS_PIN) && SPI_CS_PIN == AUX_IO4_PIN
    #error "Modbus direction pin not available, is assigned as ethernet CS!"
  #else
    #define MODBUS_DIR_AUX  7
  #endif
#endif

#if I2C_STROBE_ENABLE
  #if defined(SPI_IRQ_PIN) && SPI_IRQ_PIN == 26
    #error "I2C strobe pin not available, is assigned as ethernet IRQ!" 
  #else
    #define I2C_STROBE_PIN  26
  #endif
#endif
