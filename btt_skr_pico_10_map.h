/*
  btt_skr_pico_10_map.h - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2022 Terje Io

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

#if N_ABC_MOTORS
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "BTT SKR Pico 1.0"

#undef TRINAMIC_ENABLE
#undef TRINAMIC_UART_ENABLE
#define TRINAMIC_ENABLE 2209
#define TRINAMIC_UART_ENABLE 1
#define HAS_BOARD_INIT

// Define step pulse output pins.
#define X_STEP_PIO_PIN              11  // X_AXIS step out via PIO
#define Y_STEP_PIO_PIN              8   // Y_AXIS step out via PIO
#define Z_STEP_PIO_PIN              19  // Z_AXIS step out via PIO

// Define step direction output pins.
#define X_DIRECTION_PIN             10
#define Y_DIRECTION_PIN             5
#define Z_DIRECTION_PIN             28
#define DIRECTION_OUTMODE           GPIO_SHIFT5

// Define stepper driver enable/disable output pin.
#define X_STEPPERS_ENABLE_PORT      12
#define Y_STEPPERS_ENABLE_PORT      7
#define Z_STEPPERS_ENABLE_PORT      2

// Define homing/hard limit switch input pins.
#define LIMIT_PORT                  GPIO_IN
#define X_LIMIT_PIN                 3
#define Y_LIMIT_PIN                 4
#define Z_LIMIT_PIN                 25
#define LIMIT_INMODE                GPIO_OE

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT         GPIO_OUTPUT
#define SPINDLE_ENABLE_PIN          17
#define SPINDLE_DIRECTION_PORT      GPIO_OUTPUT
#define SPINDLE_DIRECTION_PIN       18
#define SPINDLE_OUTMODE             GPIO_OUTPUT

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT            GPIO_OUTPUT
#define SPINDLE_PWM_PIN             20

// Define flood and mist coolant enable output pins.
/*
#define COOLANT_FLOOD_PORT          GPIO_OUTPUT
#define COOLANT_FLOOD_PIN           2
#define COOLANT_MIST_PORT           GPIO_OUTPUT
#define COOLANT_MIST_PIN            3
#define COOLANT_OUTMODE             GPIO_OUTPUT
*/

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PIN                   12
#define FEED_HOLD_PIN               13
#define CYCLE_START_PIN             14
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN             15
#endif
#define CONTROL_INMODE              GPIO_MAP

// Define probe switch input pin.
#define PROBE_PORT                  GPIO_INPUT
#define PROBE_PIN                   16
