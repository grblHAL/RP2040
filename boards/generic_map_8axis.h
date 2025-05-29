/*
  generic_map_8axis.h - driver code for RP2040 ARM processors

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

// NOTE: this board map is an example for how to enable up to 8 axes,
//       it is not intended for use in a machine.

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

#if N_ABC_MOTORS > 7
#error "Axis configuration is not supported!"
#endif

#undef PROBE_ENABLE
#define PROBE_ENABLE 0

// Define step pulse output pins.
#define STEP_PORT               GPIO_PIO  // N_AXIS pin PIO SM
#define STEP_PINS_BASE          2         // N_AXIS number of consecutive pins are used by PIO

// Define step direction output pins.
#define DIRECTION_PORT          GPIO_OUTPUT
#define X_DIRECTION_PIN         10
#define Y_DIRECTION_PIN         11
#define Z_DIRECTION_PIN         12
#define DIRECTION_OUTMODE       GPIO_SHIFT10

// Define stepper driver enable/disable output pin.
#define ENABLE_PORT             GPIO_OUTPUT
#define STEPPERS_ENABLE_PIN     18

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN             19
#define Y_LIMIT_PIN             20
#define Z_LIMIT_PIN             21
#define LIMIT_INMODE            GPIO_MAP

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN             (STEP_PINS_BASE + 3)
#define M3_DIRECTION_PIN        (Z_DIRECTION_PIN + 1)
#define M3_LIMIT_PIN            (Z_LIMIT_PIN + 1)
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 1
#define M4_AVAILABLE
#define M4_STEP_PIN             (STEP_PINS_BASE + 4)
#define M4_DIRECTION_PIN        (Z_DIRECTION_PIN + 2)
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 2
#define M5_AVAILABLE
#define M5_STEP_PIN             (STEP_PINS_BASE + 5)
#define M5_DIRECTION_PIN        (Z_DIRECTION_PIN + 3)
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 3
#define M6_AVAILABLE
#define M6_STEP_PIN             (STEP_PINS_BASE + 6)
#define M6_DIRECTION_PIN        (Z_DIRECTION_PIN + 4)
#endif

#if N_ABC_MOTORS == 5
#define M7_AVAILABLE
#define M7_STEP_PIN             (STEP_PINS_BASE + 7)
#define M7_DIRECTION_PIN        (Z_DIRECTION_PIN + 5)
#endif

#define AUXOUTPUT0_PORT         GPIO_OUTPUT // Spindle PWM
#define AUXOUTPUT0_PIN          26
#define AUXOUTPUT1_PORT         GPIO_OUTPUT // Spindle direction
#define AUXOUTPUT1_PIN          27
#define AUXOUTPUT2_PORT         GPIO_OUTPUT // Spindle enable
#define AUXOUTPUT2_PIN          28   

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_PORT            GPIO_OUTPUT
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT1_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA   
#define SPINDLE_ENABLE_PIN      AUXOUTPUT0_PIN
#endif

#undef CONTROL_ENABLE
#define CONTROL_ENABLE 0

/*
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

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PIN               18
#define FEED_HOLD_PIN           19
#define CYCLE_START_PIN         20

#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT3_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         AUXINPUT1_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PIN         AUXINPUT1_PIN
#endif
*/
