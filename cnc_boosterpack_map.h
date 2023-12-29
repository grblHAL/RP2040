/*
  cnc_boosterpack_map.h - driver code for RP2040 ARM processors

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

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

#if N_ABC_MOTORS
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "CNC BoosterPack"
#define BOARD_URL "https://github.com/terjeio/CNC_Boosterpack"

#define IOEXPAND_ENABLE 1

#undef EEPROM_ENABLE
#define EEPROM_ENABLE 1
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
#define ENABLE_PORT                 GPIO_IOEXPAND
#define STEPPERS_ENABLEX_PIN        6
#define STEPPERS_ENABLEZ_PIN        0

// Define homing/hard limit switch input pins.
#define LIMIT_PORT                  GPIO_IN
#define X_LIMIT_PIN                 19
#define Y_LIMIT_PIN                 20
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
#define SPINDLE_PORT                GPIO_IOEXPAND
#define SPINDLE_DIRECTION_PIN       5
#endif

#if DRIVER_SPINDLE_ENABLE
#ifndef SPINDLE_PORT
#define SPINDLE_PORT                GPIO_IOEXPAND
#endif
#define SPINDLE_ENABLE_PIN          0
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_PORT                GPIO_IOEXPAND
#define COOLANT_FLOOD_PIN           2
#define COOLANT_MIST_PIN            3

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PIN                   12
#define FEED_HOLD_PIN               13
#define CYCLE_START_PIN             14
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN             15
#endif

// Define probe switch input pin.
#define PROBE_PORT                  GPIO_INPUT
#define PROBE_PIN                   16

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PIN              17
#endif

#define I2C_PORT                    1
#define I2C_SDA                     10
#define I2C_SCL                     11

#if SDCARD_ENABLE
#define SPI_PORT spi0
#define SD_MISO_PIN                 16
#define SD_CS_PIN                   17
#define SD_SCK_PIN                  18
#define SD_MOSI_PIN                 19
#endif

#if MPG_MODE_ENABLE
#define MODE_SWITCH_PIN             18
#endif

//I2C: 26,27
//Free: 21,21,22,28
