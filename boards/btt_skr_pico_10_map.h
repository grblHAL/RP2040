/*
  btt_skr_pico_10_map.h - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2022-2024 Terje Io

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

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#if WIFI_ENABLE || BLUETOOTH_ENABLE == 1
#error "Wireless networking is not supported!"
#endif

//#define AUX_CONTROLS_OUT

#define BOARD_NAME "BTT SKR Pico 1.0"
#define BOARD_URL "https://github.com/bigtreetech/SKR-Pico"

#define SERIAL1_PORT 1

#undef TRINAMIC_ENABLE
#undef TRINAMIC_UART_ENABLE
#define TRINAMIC_ENABLE 2209
#define TRINAMIC_UART_ENABLE 1
#define TRINAMIC_STREAM 1
#undef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 0
#define HAS_BOARD_INIT

// Define step pulse output pins.
#define STEP_PORT                   GPIO_PIO_1 // Single pin PIO SM
#define X_STEP_PIN                  11
#define Y_STEP_PIN                  6
#define Z_STEP_PIN                  19

// Define step direction output pins.
#define DIRECTION_PORT              GPIO_OUTPUT
#define DIRECTION_OUTMODE           GPIO_MAP
#define X_DIRECTION_PIN             10
#define Y_DIRECTION_PIN             5
#define Z_DIRECTION_PIN             28

// Define stepper driver enable/disable output pin.
#define ENABLE_PORT                 GPIO_OUTPUT
#define X_ENABLE_PIN                12
#define Y_ENABLE_PIN                7
#define Z_ENABLE_PIN                2

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN                 14
#define M3_DIRECTION_PIN            13
#define M3_ENABLE_PIN               15
#define M3_LIMIT_PIN                16
#else
#define RESET_PIN                   16  // E0 limit
#endif

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN                 4
#define Y_LIMIT_PIN                 3
#define Z_LIMIT_PIN                 25

#define AUXOUTPUT0_PORT             GPIO_OUTPUT // Spindle PWM
#define AUXOUTPUT0_PIN              20
#define AUXOUTPUT1_PORT             GPIO_OUTPUT // Spindle direction
#define AUXOUTPUT1_PIN              18
#define AUXOUTPUT2_PORT             GPIO_OUTPUT // Spindle enable
#define AUXOUTPUT2_PIN              17   
#define AUXOUTPUT3_PORT             GPIO_OUTPUT // Coolant flood
#define AUXOUTPUT3_PIN              21 // HB PWM
#define AUXOUTPUT4_PORT             GPIO_OUTPUT // Coolant mist
#define AUXOUTPUT4_PIN              23 // HE PWM

#define AUXOUTPUT0_PWM_PIN          29 // Servo

// Define driver spindle pins

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_PORT                GPIO_OUTPUT
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN          AUXOUTPUT2_PIN 
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN             AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN       AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE
#define COOLANT_PORT                GPIO_OUTPUT
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN           AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN            AUXOUTPUT4_PIN
#endif

#define AUXINPUT0_PIN               22 // Probe

#undef CONTROL_ENABLE
#define CONTROL_ENABLE 0

#if PROBE_ENABLE
#define PROBE_PIN                   AUXINPUT0_PIN
#endif

#if RGB_LED_ENABLE
#define NEOPIXELS_PIN               24
#endif

#if MODBUS_ENABLE
#define MODBUS_RTU_STREAM           0
#endif
