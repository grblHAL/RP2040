/*
  generic_map_4axis.h - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021-2023 Terje Io
  Copyright (c) 2021 Volksolive

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

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

// Define step pulse output pins.
#define STEP_PORT               GPIO_PIO  // N_AXIS pin PIO SM
#define STEP_PINS_BASE          2         // N_AXIS number of consecutive pins are used by PIO

// Define step direction output pins.
#define DIRECTION_PORT          GPIO_OUTPUT
#define X_DIRECTION_PIN         6
#define Y_DIRECTION_PIN         7
#define Z_DIRECTION_PIN         8
#define DIRECTION_OUTMODE       GPIO_SHIFT6

// Define stepper driver enable/disable output pin.
#define ENABLE_PORT             GPIO_OUTPUT
#define STEPPERS_ENABLE_PIN     10

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN             11
#define Y_LIMIT_PIN             12
#define Z_LIMIT_PIN             13
#define LIMIT_INMODE            GPIO_MAP

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN             (STEP_PINS_BASE + 3)
#define M3_DIRECTION_PIN        (Z_DIRECTION_PIN + 1)
#define M3_LIMIT_PIN            (Z_LIMIT_PIN + 1)
#else
#define AUXINPUT0_PIN           14
#define AUXOUTPUT0_PORT         GPIO_OUTPUT
#define AUXOUTPUT0_PIN          5
#define AUXOUTPUT1_PORT         GPIO_OUTPUT
#define AUXOUTPUT1_PIN          9
#endif

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PORT        GPIO_OUTPUT
#define SPINDLE_PWM_PIN         15
#else
#define AUXOUTPUT0_PORT         GPIO_OUTPUT
#define AUXOUTPUT0_PIN          15
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_PORT            GPIO_OUTPUT
#define SPINDLE_DIRECTION_PIN   27
#else
#define AUXOUTPUT1_PORT         GPIO_OUTPUT
#define AUXOUTPUT1_PIN          27
#endif

#if DRIVER_SPINDLE_ENABLE
#ifndef SPINDLE_PORT
#define SPINDLE_PORT            GPIO_OUTPUT
#endif
#define SPINDLE_ENABLE_PIN      26
#else
#define AUXOUTPUT2_PORT         GPIO_OUTPUT
#define AUXOUTPUT2_PIN          26   
#endif

#define AUXINPUT1_PIN           21

// Define flood and mist coolant enable output pins.
#define COOLANT_PORT            GPIO_OUTPUT
#define COOLANT_FLOOD_PIN       16
#define COOLANT_MIST_PIN        17

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PIN               18
#define FEED_HOLD_PIN           19
#define CYCLE_START_PIN         20

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         AUXINPUT1_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PIN         AUXINPUT1_PIN
#endif

// Define probe switch input pin.
#define PROBE_PIN               22

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PIN          28
#endif
