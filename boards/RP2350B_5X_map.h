/*
  RP2350B 5 Axis driver pin map

  Part of grblHAL

  Copyright (c) 2024-2025 Terje Io
  Copyright (c) 2024 PL Barrett

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

#if RP_MCU != 2350
#error "Board has a RP2350 processor!"
#endif

#define BOARD_NAME "RP23U5XBB"

#undef I2C_ENABLE
#define I2C_ENABLE    1
#if !SPINDLE_ENCODER_ENABLE
//#define SERIAL1_PORT  1
#endif

// Define step pulse output pins.
#define STEP_PORT               GPIO_PIO  // N_AXIS pin PIO SM
#define STEP_PINS_BASE          17        // N_AXIS number of consecutive pins are used by PIO

// Define step direction output pins.
#define DIRECTION_PORT          GPIO_OUTPUT
#define X_DIRECTION_PIN         12
#define Y_DIRECTION_PIN         13
#define Z_DIRECTION_PIN         14
#define DIRECTION_OUTMODE       GPIO_SHIFT12

// Define stepper driver enable/disable output pin.
#define ENABLE_PORT             GPIO_OUTPUT
#define X_ENABLE_PIN            22
#define Y_ENABLE_PIN            23
#define Z_ENABLE_PIN            24

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN             6 // pre 1.0 -> 5
#define Y_LIMIT_PIN             5 // pre 1.0 -> 6
#define Z_LIMIT_PIN             4 // pre 1.0 -> 3
#define LIMIT_INMODE            GPIO_MAP

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_DIRECTION_PIN        15
#define M3_LIMIT_PIN            3 // pre 1.0 -> 4
#define M3_ENABLE_PIN           25
#endif

#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_DIRECTION_PIN        16
#define M4_LIMIT_PIN            2
#define M4_ENABLE_PIN           26
#else
#define AUXINPUT6_PIN           2 // M4_LIMIT_PIN
#endif

#define AUXOUTPUT0_PORT         GPIO_OUTPUT
#define AUXOUTPUT0_PIN          36
#if RGB_LED_ENABLE
#define NEOPIXELS_PIN           37
#else
#define AUXOUTPUT1_PORT         GPIO_OUTPUT
#define AUXOUTPUT1_PIN          37
#endif
#ifndef SERIAL1_PORT
#define AUXOUTPUT2_PORT         GPIO_OUTPUT
#define AUXOUTPUT2_PIN          38
#endif
#define AUXOUTPUT3_PORT         GPIO_OUTPUT  // Spindle enable
#define AUXOUTPUT3_PIN          33
#define AUXOUTPUT4_PORT         GPIO_OUTPUT  // Spindle PWM
#define AUXOUTPUT4_PIN          35
#define AUXOUTPUT5_PORT         GPIO_OUTPUT  // Spindle direction
#define AUXOUTPUT5_PIN          34
#define AUXOUTPUT6_PORT         GPIO_OUTPUT  // Coolant flood
#define AUXOUTPUT6_PIN          39
#define AUXOUTPUT7_PORT         GPIO_OUTPUT  // Coolant mist
#define AUXOUTPUT7_PIN          40

//#define AUXOUTPUT0_PWM_PIN      29 // Servo

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_PORT            GPIO_OUTPUT
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT3_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT5_PIN
#endif

#if COOLANT_ENABLE
#define COOLANT_PORT            GPIO_OUTPUT
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT6_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT7_PIN
#endif

//

#if SPINDLE_ENCODER_ENABLE
#define SPINDLE_PULSE_PIN       29  // Must be an odd pin
#define SPINDLE_INDEX_PIN       28
#else
#define AUXINPUT0_PIN           29
#define AUXINPUT1_PIN           28
#endif

#ifndef SERIAL1_PORT
#define AUXINPUT2_PIN           27
#endif

#define AUXINPUT3_PIN           7  // Probe
#define AUXINPUT4_PIN           8  // Safety door or motor fault
#define AUXINPUT5_PIN           32 // I2C strobe pin
#define AUXINPUT7_PIN           11 // Reset/EStop
#define AUXINPUT8_PIN           10 // Feed hold
#define AUXINPUT9_PIN           9  // Cycle start

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN               AUXINPUT7_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PIN           AUXINPUT8_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN         AUXINPUT9_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT3_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         AUXINPUT4_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PIN         AUXINPUT4_PIN // set Door as alternate Motor Fault input
#elif TOOLSETTER_ENABLE
#define TOOLSETTER_PIN          AUXINPUT4_PIN // set Door as alternate Toolsetter input
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PIN          AUXINPUT5_PIN
#endif

#if SDCARD_ENABLE || ETHERNET_ENABLE

#define SPI_PORT                1
#define SPI_SCK_PIN             46
#define SPI_MOSI_PIN            43
#define SPI_MISO_PIN            44

#if SDCARD_ENABLE
#define SD_CS_PIN               45
#endif

#if ETHERNET_ENABLE
#define SPI_CS_PIN              41
#define SPI_IRQ_PIN             42
//#define SPI_RST_PORT          43
#endif

#endif // SDCARD_ENABLE || ETHERNET_ENABLE

#if I2C_ENABLE
#define I2C_PORT                1
#define I2C_SDA                 30    
#define I2C_SCL                 31
#endif

#ifdef SERIAL1_PORT
#define UART_1_RX_PIN           27
#define UART_1_TX_PIN           36
#endif
