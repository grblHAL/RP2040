/*
  RP2350B 5 Axis driver pin map

  Part of grblHAL

  Copyright (c) 2021-2024 Terje Io
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

#if PICO_PLATFORM != rp2350
#error "Board has a RP2350 processor!"
#endif

#define BOARD_NAME "RP23U5XBB"

#undef I2C_ENABLE
#define I2C_ENABLE    1
//#define SERIAL1_PORT  1

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
#define X_LIMIT_PIN             5 // 1.0 -> 6
#define Y_LIMIT_PIN             6 // 1.0 -> 5
#define Z_LIMIT_PIN             3 // 1.0 -> 4
#define LIMIT_INMODE            GPIO_MAP

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_DIRECTION_PIN        31
#define M3_LIMIT_PIN            4 // 1.0 -> 3
#define M3_ENABLE_PIN           41
#endif

#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_DIRECTION_PIN        32
#define M4_LIMIT_PIN            2
#define M4_ENABLE_PIN           42
#else
#define AUXINPUT6_PIN			      2 // M4_LIMIT_PIN
#endif

#define AUXOUTPUT0_PORT         GPIO_OUTPUT
#define AUXOUTPUT0_PIN          38
#define AUXOUTPUT1_PORT         GPIO_OUTPUT
#define AUXOUTPUT1_PIN          37

//#define AUXOUTPUT3_PORT         GPIO_OUTPUT
#define AUXOUTPUT3_PIN          33  // Spindle enable

//#define AUXOUTPUT4_PORT         GPIO_OUTPUT
#define AUXOUTPUT4_PIN			    35  // Spindle PWM

//#define AUXOUTPUT5_PORT         GPIO_OUTPUT
#define AUXOUTPUT5_PIN			    34  // Spindle direction
//#define AUXOUTPUT0_PWM_PIN      29 // Servo

#if DRIVER_SPINDLE_ENABLE

#define SPINDLE_PORT            GPIO_OUTPUT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT3_PIN

#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PORT        GPIO_OUTPUT
#define SPINDLE_PWM_PIN         AUXOUTPUT4_PIN
#else
#define AUXOUTPUT4_PORT         GPIO_OUTPUT
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT5_PIN
#else
#define AUXOUTPUT5_PORT         GPIO_OUTPUT
#endif

#else
#define AUXOUTPUT3_PORT         GPIO_OUTPUT
#endif // DRIVER_SPINDLE_ENABLE

#define COOLANT_PORT            GPIO_OUTPUT
#define COOLANT_FLOOD_PIN       39
#define COOLANT_MIST_PIN        40

//

#define AUXINPUT0_PIN           29
#define AUXINPUT1_PIN           28
#define AUXINPUT3_PIN           7   // Probe
#define AUXINPUT4_PIN           8   // Safety door or motor fault
#define AUXINPUT5_PIN           32  // I2C strobe pin

#define RESET_PIN               11
#define FEED_HOLD_PIN           10
#define CYCLE_START_PIN         9

#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT3_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         AUXINPUT4_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PIN         AUXINPUT4_PIN // set Door as alternate Motor Fault input
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
#define	I2C_SDA				          30	
#define	I2C_SCL				          31
#endif

#ifdef SERIAL1_PORT
#define UART_1_RX_PIN				    27
#define	UART_1_TX_PIN				    36
#else
#define AUXINPUT2_PIN           27
#define AUXOUTPUT2_PORT         GPIO_OUTPUT
#define AUXOUTPUT2_PIN          36
#endif
