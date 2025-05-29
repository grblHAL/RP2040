/*
  pico_cnc_map.h - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021-2024 Terje Io

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

#include <stdint.h>

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "PicoCNC"
#define BOARD_URL "https://github.com/phil-barrett/PicoCNC"
#define USE_EXPANDERS
#define HAS_BOARD_INIT

/*
SR16 bit mappings:
  x_enable    0
  y_enable    1
  z_enable    2
  m3_enable   3
  spindle_dir 4
  spindle_ena 5
  mist_ena    6
  flood_ena   7
  aux0_out    8
  aux1_out    9
  aux2_out    10
  aux3_out    11
  aux4_out    12
  aux5_out    13
  aux6_out    14
  spi_reset   15
*/

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
#define ENABLE_PORT         EXPANDER_PORT

#define X_ENABLE_PIN        0
#define Y_ENABLE_PIN        1
#define Z_ENABLE_PIN        2

#define AUX_N_OUT           8
#define AUX_OUT_MASK        0xFF

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN         0 // Not referenced by driver code
#define M3_DIRECTION_PIN    0 // Not referenced by driver code
#define M3_LIMIT_PIN        3
#define M3_ENABLE_PIN       3
#endif

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN         6
#define Y_LIMIT_PIN         5
#define Z_LIMIT_PIN         4

#define SPINDLE_PORT        EXPANDER_PORT
#define COOLANT_PORT        EXPANDER_PORT

// Define spindle PWM output pin.

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT    GPIO_OUTPUT
#define SPINDLE_PWM_PIN     27
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN    5
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN  4
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN           7
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN            6
#endif

// Define aux I/O

#define AUX_IO0_PIN         10
#define AUX_IO1_PIN         11
#define AUX_IO2_PIN         12
#define AUX_IO3_PIN         13
#define AUX_IO4_PIN         2 // Modbus direction or Ethernet CS

#if !(SDCARD_ENABLE || ETHERNET_ENABLE) 
#define AUXINPUT0_PIN       AUX_IO0_PIN
#define AUXINPUT1_PIN       AUX_IO1_PIN
#define AUXINPUT2_PIN       AUX_IO2_PIN
#if MPG_ENABLE != 1
#define AUXOUTPUT0_PWM_PIN  AUX_IO3_PIN
#endif 
#endif
#define AUXINPUT3_PIN       9
#define AUXINPUT4_PIN       28
#define AUXINPUT6_PIN       22 // Reset/EStop
#define AUXINPUT7_PIN       7  // Feed hold
#define AUXINPUT8_PIN       8  // Cycle start

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN           AUXINPUT6_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PIN       AUXINPUT7_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN     AUXINPUT8_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN           AUXINPUT4_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN     AUXINPUT3_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PIN     AUXINPUT3_PIN
#elif MOTOR_WARNING_ENABLE
#define MOTOR_WARNING_PIN   AUXINPUT3_PIN
#endif

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
    #define AUXOUTPUT0_PORT GPIO_OUTPUT
    #define AUXOUTPUT0_PIN  AUX_IO4_PIN
  #else
    #define SPI_CS_PIN      AUX_IO4_PIN
  #endif
    #define SPI_IRQ_PIN       26
    #define SPI_RST_PORT      EXPANDER_PORT
    #define SPI_RST_PIN       15
#else
    #define AUXINPUT5_PIN     26
  #if RGB_LED_ENABLE
    #define NEOPIXELS_PIN     AUX_IO4_PIN
  #else
    #define AUXOUTPUT0_PORT   GPIO_OUTPUT
    #define AUXOUTPUT0_PIN    AUX_IO4_PIN
  #endif
#endif

#if !(WIFI_ENABLE || BLUETOOTH_ENABLE == 1 || defined(NEOPIXELS_PIN))
#define LED_G_PIN           25
#endif

#if SDCARD_ENABLE || ETHERNET_ENABLE
#define SPI_PORT            1
#define SPI_SCK_PIN         AUX_IO0_PIN
#define SPI_MOSI_PIN        AUX_IO1_PIN
#define SPI_MISO_PIN        AUX_IO2_PIN
#endif

#if MPG_ENABLE == 1
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
    #define MODBUS_DIR_AUX  AUXOUTPUT0_PIN
  #endif
#endif

#if I2C_STROBE_ENABLE
#ifdef AUXINPUT5_PIN
#define I2C_STROBE_PIN    AUXINPUT5_PIN
#else
#error "I2C strobe pin not available, is assigned as Ethernet IRQ!" 
#endif
#endif
