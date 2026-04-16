/*
  FlexiHAL 2350 driver pin map

  Part of grblHAL

  Copyright (c) 2024 Terje Io
  Copyright (c) 2024 PL Barrett
  Copyright (c) 2025 Expatria Technologies Inc.
  Copyright (c) 2026 Mitchell Grams

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

#if N_ABC_MOTORS > 3
#error "Axis configuration is not supported!"
#endif

#if RP_MCU != 2350
#error "Board has a RP2350 processor!"
#endif

#define BOARD_NAME "FLEXIHAL2350"
#define BOARD_URL "https://github.com/Expatria-Technologies/FlexiHAL_2350"

#define HAS_BOARD_INIT      1

// Define support for FLEXGPIO RP2040 IO expander
#if !FLEXGPIO_ENABLE
#error "Board requires FLEXGPIO expander to be enabled!"
#endif

#define USE_EXPANDERS       1

#ifdef PICOHAL_IO_ENABLE // need to increase if also using picoHAL IO expander
#define IOX_PIN_COUNT       32 
#else
#define IOX_PIN_COUNT       48
#endif

#undef I2C_ENABLE
#undef EEPROM_ENABLE
#define I2C_ENABLE    1
#define EEPROM_ENABLE 128

#define LITTLEFS_ENABLE 0

// Define step pulse output pins.
#define STEP_PORT               GPIO_PIO_1 // Single pin PIO SM
#define X_STEP_PIN              12
#define Y_STEP_PIN              14
#define Z_STEP_PIN              16

// Define step direction output pins.
#define DIRECTION_PORT          GPIO_OUTPUT
#define DIRECTION_OUTMODE       GPIO_MAP
#define X_DIRECTION_PIN         13
#define Y_DIRECTION_PIN         15
#define Z_DIRECTION_PIN         17

// Define stepper driver enable/disable output pins.
#define ENABLE_PORT             EXPANDER_PORT
#define X_ENABLE_PIN            29 //RP2040 pin
#define Y_ENABLE_PIN            28 //RP2040 pin
#define Z_ENABLE_PIN            27 //RP2040 pin

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN             38
#define Y_LIMIT_PIN             37
#define Z_LIMIT_PIN             36
#define LIMIT_INMODE            GPIO_MAP

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN             18
#define M3_DIRECTION_PIN        19
#define M3_LIMIT_PIN            35
#define M3_ENABLE_PIN           26 //RP2040 pin
#endif

#if N_ABC_MOTORS >= 2
#define M4_AVAILABLE
#define M4_STEP_PIN             20
#define M4_DIRECTION_PIN        21
#define M4_LIMIT_PIN            34
#define M4_ENABLE_PIN           25 //RP2040 pin
#endif

#if N_ABC_MOTORS == 3
#define M5_AVAILABLE
#define M5_STEP_PIN             22
#define M5_DIRECTION_PIN        23
#define M5_LIMIT_PIN            34
#define M5_ENABLE_PIN           24 //RP2040 pin
#endif

#if COOLANT_ENABLE
#define COOLANT_PORT            EXPANDER_PORT
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        13 //RP2040 pin
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       14 //RP2040 pin
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_PORT               EXPANDER_PORT
#define SPINDLE_ENABLE_PORT        EXPANDER_PORT
#define SPINDLE_DIRECTION_PORT     EXPANDER_PORT
#define SPINDLE_PWM_PORT           GPIO_OUTPUT // Spindle PWM
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      11 //RP2040 pin
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         26
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   12 //RP2040 pin
#endif

#define AUXINPUT0_PIN           47  // Encoder 2
#define AUXINPUT1_PIN           46  // Encoder 2
#define AUXINPUT2_PIN           45  // Encoder 2

#define AUXINPUT3_PIN           9   // Encoder 1
#define AUXINPUT4_PIN           10  // Encoder 1
#define AUXINPUT5_PIN           11  // Encoder 1

#define AUXINPUT6_PIN           24  // Reset / HALT
#define AUXINPUT7_PIN           27  // Feed Hold / HD
#define AUXINPUT8_PIN           32  // Safety door / DR
#define AUXINPUT9_PIN           30  // Cycle Start / RN

#define AUXINPUT10_PIN          8   // I2C strobe pin
#define AUXINPUT11_PIN          31  // Expander MCU_IRQ Pin
//#define AUXINPUT12_PIN        39  // Probe

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN               AUXINPUT6_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PIN           AUXINPUT7_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN         AUXINPUT9_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         AUXINPUT8_PIN
#endif

#if TOOLSETTER_ENABLE
#define TOOLSETTER_PORT         EXPANDER_PORT
#define TOOLSETTER_PIN          3 //RP2040 pin
#endif

#if PROBE_ENABLE
#define PROBE_PORT              EXPANDER_PORT
#define PROBE_PIN               4 //RP2040 pin
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PIN          AUXINPUT10_PIN
#endif

#if FLEXGPIO_IRQ_ENABLE
#define FLEXGPIO_IRQ_PIN        11 //AUXINPUT NUMBER
#endif

// /Define per axis fault pins on expander
#define MOTOR_FAULT_PORT            EXPANDER_PORT
#define X_MOTOR_FAULT_PIN           5  //RP2040 pin
#define Y_MOTOR_FAULT_PIN           6  //RP2040 pin
#define Z_MOTOR_FAULT_PIN           7  //RP2040 pin
#if N_ABC_MOTORS > 0
  #define M3_MOTOR_FAULT_PIN        8  //RP2040 pin
#endif
#if N_ABC_MOTORS >= 2
  #define M4_MOTOR_FAULT_PIN        9  //RP2040 pin
#endif
#if N_ABC_MOTORS == 3
  #define M5_MOTOR_FAULT_PIN        10 //RP2040 pin
#endif

#if SDCARD_ENABLE || ETHERNET_ENABLE

#define SPI_PORT                0
#define SPI_SCK_PIN             2
#define SPI_MOSI_PIN            3
#define SPI_MISO_PIN            0
#define SD_CS_PIN               44

#if ETHERNET_ENABLE
#define SPI_CS_PIN              33
#define SPI_IRQ_PIN             25
#define WIZNET_CS_PIN           SPI_CS_PIN
#endif

#endif // SDCARD_ENABLE || ETHERNET_ENABLE

#if I2C_ENABLE
#define I2C_PORT                1
#define I2C_SDA                 6    
#define I2C_SCL                 7
#endif

// UART 0 (MODBUS)
#if MODBUS_ENABLE
#define MODBUS_RTU_STREAM       0
//#define UART_PORT               uart0
#define UART_TX_PIN             28
#define UART_RX_PIN             29
#endif

// UART 1 (RPI)
#define SERIAL1_PORT            1
//#define UART_1_PORT             uart1
#define UART_1_TX_PIN           4
#define UART_1_RX_PIN           5