/*
  picobob_map.h - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021-2023 Andrew Marles

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

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "PicoBOB"
#define BOARD_URL "https://github.com/Expatria-Technologies/PicoBOB"

// Define step pulse output pins.
#define STEP_PORT             GPIO_PIO  // N_AXIS pin PIO SM
#define STEP_PINS_BASE        17        // N_AXIS number of consecutive pins are used by PIO

// Define step direction output pins.
#define DIRECTION_PORT        GPIO_OUTPUT
#define DIRECTION_OUTMODE     GPIO_MAP
#define X_DIRECTION_PIN       9
#define Y_DIRECTION_PIN       10
#define Z_DIRECTION_PIN       11

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_DIRECTION_PIN      12
#define M3_LIMIT_PIN          5
#endif

//M4 pins  Note that M3 and M4 must use the same limit pin.
#if N_ABC_MOTORS > 1
#define M4_AVAILABLE
#define M4_DIRECTION_PIN      13
#define M4_LIMIT_PIN          5
#endif

//Define stepper driver enable/disable output pin.  This is not used on PicoBOB.

// Define homing/hard limit switch input pins.  Currently configured so that X and Z limit pins are shared.
#define X_LIMIT_PIN           2
#define Y_LIMIT_PIN           1
#define Z_LIMIT_PIN           2

// Define driver spindle pins
// No direction signal on the Mach3 BOB.
// Spindle relay control is shared with B direction port, only one can be enabled at a time!
#define AUXOUTPUT0_PORT       GPIO_OUTPUT // Spindle PWM
#define AUXOUTPUT0_PIN        14
#ifndef M4_DIRECTION_PIN
#define AUXOUTPUT1_PORT       GPIO_OUTPUT // Spindle enable
#define AUXOUTPUT1_PIN        13
#endif
#define AUXOUTPUT2_PORT       GPIO_OUTPUT // Coolant flood
#define AUXOUTPUT2_PIN        16

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_PORT            GPIO_OUTPUT
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if (DRIVER_SPINDLE_ENABLE & SPINDLE_ENA) && defined(AUXOUTPUT1_PIN)
#define SPINDLE_ENABLE_PIN      AUXOUTPUT1_PIN
#endif

//Stepper enable is replaced with coolant control

#if COOLANT_ENABLE
#define COOLANT_PORT            GPIO_OUTPUT
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT2_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#undef COOLANT_ENABLE
#ifdef COOLANT_FLOOD_PIN
#define COOLANT_ENABLE COOLANT_FLOOD
#else
#define COOLANT_ENABLE 0
#endif
#endif

#define AUXINPUT0_PIN         4
#define AUXINPUT1_PIN         3  // Reset

// Define user-control controls (cycle start, reset, feed hold) input pins.  Only Estop is supported on the Mach3 BOB.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN             AUXINPUT1_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN             AUXINPUT0_PIN
#endif
