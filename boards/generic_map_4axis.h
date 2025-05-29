/*
  generic_map_4axis.h - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io
  Copyright (c) 2021 Volksolive

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

#define AUXOUTPUT2_PORT         GPIO_OUTPUT // Spindle PWM
#define AUXOUTPUT2_PIN          15
#define AUXOUTPUT3_PORT         GPIO_OUTPUT // Spindle direction
#define AUXOUTPUT3_PIN          27
#define AUXOUTPUT4_PORT         GPIO_OUTPUT // Spindle enable
#define AUXOUTPUT4_PIN          26   
#define AUXOUTPUT5_PORT         GPIO_OUTPUT // Coolant flood
#define AUXOUTPUT5_PIN          16   
#define AUXOUTPUT6_PORT         GPIO_OUTPUT // Coolant mist
#define AUXOUTPUT6_PIN          17   

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_PORT            GPIO_OUTPUT
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT3_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA   
#define SPINDLE_ENABLE_PIN      AUXOUTPUT4_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE
#define COOLANT_PORT            GPIO_OUTPUT
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT5_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT6_PIN
#endif

#define AUXINPUT1_PIN           21
#define AUXINPUT2_PIN           28
#define AUXINPUT3_PIN           22 // Probe
#define AUXINPUT4_PIN           18 // Reset/EStop
#define AUXINPUT5_PIN           19 // Feed hold
#define AUXINPUT6_PIN           20 // Cycle start

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN               AUXINPUT4_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PIN           AUXINPUT5_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN         AUXINPUT6_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT3_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         AUXINPUT1_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PIN         AUXINPUT1_PIN
#endif
