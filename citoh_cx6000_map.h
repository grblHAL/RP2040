/*
  citoh_cx6000_map.h - driver code for RP2040 ARM processors

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

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#undef KEYPAD_ENABLE
#define KEYPAD_ENABLE 2

#define BOARD_NAME "C.ITOH CX-6000 Plotter"
#define HAS_BOARD_INIT
#define HAS_IOPORTS

// Define step pulse output pins.
#define X_STEP_PIO_PIN         4   // X_AXIS step out via PIO
#define Y_STEP_PIO_PIN         7   // Y_AXIS step out via PIO

// Define step direction output pins.
#define DIRECTION_PORT          GPIO_OUTPUT
#define X_DIRECTION_PIN         3
#define Y_DIRECTION_PIN         6
#define Z_DIRECTION_PIN         15
#define DIRECTION_OUTMODE       GPIO_MAP

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIO_OUTPUT
#define STEPPERS_ENABLE_PIN     5
#define STEPPERS_ENABLE_MASK    STEPPERS_ENABLE_BIT
#define STEPPERS_ENABLE_OUTMODE GPIO_DIRECT

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN             13
#define Y_LIMIT_PIN             11
#define Z_LIMIT_PIN             11
#define LIMIT_INMODE            GPIO_MAP

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIO_OUTPUT
#define SPINDLE_ENABLE_PIN      17

#define AUX_INPUT0_PIN          18 // Y-
#define AUX_INPUT1_PIN          19 // Y+
#define AUX_INPUT2_PIN          20 // X-
#define AUX_INPUT3_PIN          21 // X+
#define AUX_INPUT4_PIN          26 // Pen U/D
#define AUX_INPUT5_PIN          22 // Online

#define AUXOUTPUT0_PORT         GPIO_OUTPUT
#define AUXOUTPUT0_PIN          8  // Pen LED
#define AUXOUTPUT1_PORT         GPIO_OUTPUT
#define AUXOUTPUT1_PIN          28 // Alert LED
#define AUXOUTPUT2_PORT         GPIO_OUTPUT
#define AUXOUTPUT2_PIN          27 // Reset front panel

#define RESET_PIN               1
#define PROBE_PIN               1
#define SPINDLE_PWM_PORT        GPIO_OUTPUT
#define SPINDLE_PWM_PIN         14
