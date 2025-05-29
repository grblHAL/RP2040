/*
  cnc_boosterpack_map.h - driver code for RP2040 ARM processors

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

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

#if N_ABC_MOTORS
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "CNC BoosterPack"
#define BOARD_URL "https://github.com/terjeio/CNC_Boosterpack"

#define USE_EXPANDERS
#if !PCA9654E_ENABLE
#error "This board uses PCA9654E I/O expander, enable it in my_machine.h!"
#endif

//#undef EEPROM_ENABLE
//#define EEPROM_ENABLE 1
#undef I2C_ENABLE
#define I2C_ENABLE 1

// Define step pulse output pins.
#define STEP_PORT                   GPIO_PIO  // N_AXIS pin PIO SM
#define STEP_PINS_BASE              2 // N_AXIS number of consecutive pins are used by PIO

// Define step direction output pins.
#define DIRECTION_PORT              GPIO_OUTPUT
#define X_DIRECTION_PIN             5
#define Y_DIRECTION_PIN             6
#define Z_DIRECTION_PIN             7
#define DIRECTION_OUTMODE           GPIO_SHIFT5

// Define stepper driver enable/disable output pin.
#define ENABLE_PORT                 EXPANDER_PORT
#define XY_ENABLE_PIN               6
#define Z_ENABLE_PIN                0

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN                 8
#define Y_LIMIT_PIN                 9
#define Z_LIMIT_PIN                 10
#define LIMIT_INMODE                GPIO_OE

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PORT            GPIO_OUTPUT
#define SPINDLE_PWM_PIN             11
#else
#define AUXOUTPUT0_PORT             GPIO_OUTPUT
#define AUXOUTPUT0_PIN              11
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_PORT                EXPANDER_PORT
#define SPINDLE_DIRECTION_PIN       5
#endif

#if DRIVER_SPINDLE_ENABLE
#ifndef SPINDLE_PORT
#define SPINDLE_PORT                EXPANDER_PORT
#endif
#define SPINDLE_ENABLE_PIN          7
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_PORT                EXPANDER_PORT
#define COOLANT_FLOOD_PIN           2
#define COOLANT_MIST_PIN            3

// Define user-control controls (cycle start, reset, feed hold) input pins.

#define AUXINPUT0_PIN               15
#define AUXINPUT1_PIN               17
#define AUXINPUT2_PIN               18
#define AUXINPUT3_PIN               15 // Probe
#define AUXINPUT4_PIN               12 // Reset/EStop
#define AUXINPUT5_PIN               13 // Feed hold
#define AUXINPUT6_PIN               14 // Cycle start

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN                   AUXINPUT4_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PIN               AUXINPUT5_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN             AUXINPUT6_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN                   AUXINPUT3_PIN
#endif

#if MPG_MODE_ENABLE
#define MODE_SWITCH_PIN             AUXINPUT2_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PIN              AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN             AUXINPUT0_PIN
#endif

#define I2C_PORT                    1
#define I2C_SDA                     26
#define I2C_SCL                     27

#if SDCARD_ENABLE
#define SPI_PORT spi0
#define SD_MISO_PIN                 16
#define SD_CS_PIN                   17
#define SD_SCK_PIN                  18
#define SD_MOSI_PIN                 19
#endif

//I2C: 26,27
//Free: 20,21,22,28
