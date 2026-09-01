/*
  bolangsk_map.h - 5 motors controller pin map

  Part of grblHAL

  Copyright (c) 2026 Terje Io

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

#if RP_MCU != 2350
#error "Board has a RP2350 processor!"
#endif

#define BOARD_NAME "Bolangsk"

//UART_0_RX_PIN 0 connected to LCD pin 3 on the board, micro jst gh 1.25        
//UART_0_TX_PIN 1 connected to LCD pin 2 on the board, micro jst gh 1.25 

#undef SERIAL1_PORT
#define SERIAL1_PORT            1
#define UART_1_RX_PIN           7 //EXT U235 on the board micro jst gh 1.25
#define UART_1_TX_PIN           6 //EXT U235 on the board micro jst gh 1.25
#undef SERIAL2_PORT
#define SERIAL2_PORT            1 // PIO based, RX only
#define UART_2_RX_PIN           27

#if MODBUS_ENABLE && (MODBUS_ENABLE & MODBUS_RTU_ENABLED)
#define MODBUS_RTU_STREAM 1
#endif

#if MPG_ENABLE && !defined(MPG_STREAM)
#define MPG_STREAM 2
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

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN             23
#define Y_LIMIT_PIN             24
#define Z_LIMIT_PIN             25
#define LIMIT_INMODE            GPIO_MAP

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_DIRECTION_PIN        15 //A|D pin 
#define M3_LIMIT_PIN            26 //LIMIT A pin
#endif

#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_DIRECTION_PIN        16 //B|D pin
#define M4_LIMIT_PIN            30 //LIMIT B pin
#endif

#define AUXOUTPUT0_PORT         GPIO_OUTPUT  //out0
#define AUXOUTPUT0_PIN          43
#define AUXOUTPUT1_PORT         GPIO_OUTPUT  //out1
#define AUXOUTPUT1_PIN          42
#define AUXOUTPUT2_PORT         GPIO_OUTPUT  //out2
#define AUXOUTPUT2_PIN          41
#define AUXOUTPUT3_PORT         GPIO_OUTPUT  //out3
#define AUXOUTPUT3_PIN          40
#define AUXOUTPUT4_PORT         GPIO_OUTPUT  // Spindle enable
#define AUXOUTPUT4_PIN          47
#define AUXOUTPUT5_PORT         GPIO_OUTPUT  // Spindle PWM
#define AUXOUTPUT5_PIN          22
#define AUXOUTPUT6_PORT         GPIO_OUTPUT  // Spindle direction
#define AUXOUTPUT6_PIN          46
#define AUXOUTPUT7_PORT         GPIO_OUTPUT  // Coolant flood
#define AUXOUTPUT7_PIN          44
#define AUXOUTPUT8_PORT         GPIO_OUTPUT  // Coolant mist
#define AUXOUTPUT8_PIN          45

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_PORT            GPIO_OUTPUT
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT5_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT6_PIN
#endif

#if COOLANT_ENABLE
#define COOLANT_PORT            GPIO_OUTPUT
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT7_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT8_PIN
#endif

#define AUXINPUT0_PIN           39 //in0
#define AUXINPUT1_PIN           33 //in1
#define AUXINPUT2_PIN           32 //in2
#define AUXINPUT3_PIN           31 //in3
#if SPINDLE_ENCODER_ENABLE
#define SPINDLE_PULSE_PIN       29  // Must be an odd pin
#define SPINDLE_INDEX_PIN       28
#else
#define AUXINPUT4_PIN           29 //Spindle|pulse pin on the board
#define AUXINPUT5_PIN           28 //Spindle|index pin on the board
#endif
#define AUXINPUT6_PIN           34 // Probe
#define AUXINPUT7_PIN           35 // Reset
#define AUXINPUT8_PIN           38 // EStop
#define AUXINPUT9_PIN           37 // Feed hold
#define AUXINPUT10_PIN          36 // Cycle start

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN               AUXINPUT8_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PIN           AUXINPUT9_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN         AUXINPUT10_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT6_PIN
#endif

#if SDCARD_ENABLE
#define SPI_PORT                0
#define SPI_SCK_PIN             2
#define SPI_MOSI_PIN            3
#define SPI_MISO_PIN            4
#define SD_CS_PIN               5
#endif // SDCARD_ENABLE

#if I2C_ENABLE
#define I2C_PORT                0
#define I2C_SDA                 8
#define I2C_SCL                 9
#endif
