/*
  citoh_cx6000_map.h - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2022-2023 Terje Io

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
#define KEYPAD_STREAM 2 // Will deliberately fail stream connect!

#define BOARD_NAME "C.ITOH CX-6000 Plotter"
#define HAS_BOARD_INIT

// Define step pulse output pins.
#define STEP_PORT               GPIO_PIO_1 // Single pin PIO SM
#define X_STEP_PIN              4
#define Y_STEP_PIN              7
#define Z_STEP_PIN              12

// Define step direction output pins.
#define DIRECTION_PORT          GPIO_OUTPUT
#define DIRECTION_OUTMODE       GPIO_MAP
#define X_DIRECTION_PIN         3
#define Y_DIRECTION_PIN         6
#define Z_DIRECTION_PIN         15

// Define stepper driver enable/disable output pin.
#define ENABLE_PORT             GPIO_OUTPUT
#define STEPPERS_ENABLE_PIN     5

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN             13
#define Y_LIMIT_PIN             11
#define Z_LIMIT_PIN             11
#define LIMIT_INMODE            GPIO_MAP

#define AUXOUTPUT0_PORT         GPIO_OUTPUT // Pen LED
#define AUXOUTPUT0_PIN          8
#define AUXOUTPUT1_PORT         GPIO_OUTPUT // Alert LED
#define AUXOUTPUT1_PIN          28
#define AUXOUTPUT2_PORT         GPIO_OUTPUT // Reset front panel
#define AUXOUTPUT2_PIN          27
#define AUXOUTPUT3_PIN          GPIO_OUTPUT // Spindle PWM
#define AUXOUTPUT3_PIN          14

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_PORT            GPIO_OUTPUT
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT3_PIN 
#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_PORT            GPIO_OUTPUT
#define SPINDLE_ENABLE_PIN      17

#define AUXINPUT0_PIN           18 // Y-
#define AUXINPUT1_PIN           19 // Y+
#define AUXINPUT2_PIN           20 // X-
#define AUXINPUT3_PIN           21 // X+
#define AUXINPUT4_PIN           26 // Pen U/D
#define AUXINPUT5_PIN           22 // Online
#define AUXINPUT6_PIN           12 // Probe
#define AUXINPUT7_PIN           10 // Reset/EStop

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN               AUXINPUT7_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT6_PIN
#endif

#if !USB_SERIAL_CDC
#define RTS_PIN                 2
#endif
