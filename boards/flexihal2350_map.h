/*
  FlexiHAL 2350 driver pin map

  Part of grblHAL

  Copyright (c) 2024 Terje Io
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

#if N_ABC_MOTORS > 3
#error "Axis configuration is not supported!"
#endif

#if RP_MCU != 2350
#error "Board has a RP2350 processor!"
#endif

#define BOARD_NAME "FLEXIHAL2350"
#define HAS_BOARD_INIT 1

#undef I2C_ENABLE
#undef EEPROM_ENABLE
#define I2C_ENABLE    1
#define EEPROM_ENABLE 2

//#define I2C_STROBE_ENABLE 1
//#define SERIAL1_PORT  0

#define FLEXGPIO_ENABLE 1

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
#define ENABLE_PORT             GPIO_IOEXPAND
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
#define M5_ENABLE_PIN           24 //RP2040 pin
#endif

#define AUXOUTPUT0_PORT         GPIO_IOEXPAND
#define AUXOUTPUT0_PIN          23 //RP2040 pin
#define AUXOUTPUT1_PORT         GPIO_IOEXPAND
#define AUXOUTPUT1_PIN          22 //RP2040 pin
#define AUXOUTPUT2_PORT         GPIO_IOEXPAND
#define AUXOUTPUT2_PIN          21 //RP2040 pin
#define AUXOUTPUT3_PORT         GPIO_IOEXPAND
#define AUXOUTPUT3_PIN          20 //RP2040 pin
#define AUXOUTPUT4_PORT         GPIO_IOEXPAND
#define AUXOUTPUT4_PIN          19 //RP2040 pin
#define AUXOUTPUT5_PORT         GPIO_IOEXPAND
#define AUXOUTPUT5_PIN          18 //RP2040 pin
#define AUXOUTPUT6_PORT         GPIO_IOEXPAND
#define AUXOUTPUT6_PIN          17 //RP2040 pin
#define AUXOUTPUT7_PORT         GPIO_IOEXPAND
#define AUXOUTPUT7_PIN          16 //RP2040 pin

#if COOLANT_ENABLE
#define COOLANT_PORT            GPIO_IOEXPAND
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        13 //RP2040 pin
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       14 //RP2040 pin
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_PORT               GPIO_IOEXPAND
#define SPINDLE_ENABLE_PORT        GPIO_IOEXPAND
#define SPINDLE_DIRECTION_PORT     GPIO_IOEXPAND
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


#define AUXINPUT0_PIN           47  //Encoder 2
#define AUXINPUT1_PIN           46  //Encoder 2
#define AUXINPUT2_PIN           45  //Encoder 2

#define AUXINPUT3_PIN           9  //Encoder 1
#define AUXINPUT4_PIN           10  //Encoder 1
#define AUXINPUT5_PIN           11  //Encoder 1

#define AUXINPUT6_PIN           32  // Safety door
#define AUXINPUT7_PIN           8  // I2C strobe pin
#define AUXINPUT8_PIN           32  // Safety door
#define AUXINPUT9_PIN           8  // I2C strobe pin ***WRONG!!!***

#define AUXINPUT10_PIN           31  // Motor Alarm
//#define AUXINPUT11_PIN           39  // Probe

#define RESET_PIN               24
#define FEED_HOLD_PIN           27
#define CYCLE_START_PIN         30

#if PROBE_ENABLE
#define PROBE_PORT              GPIO_INPUT
#define PROBE_PIN               39
#endif

#if MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PIN        31
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         AUXINPUT6_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PIN          AUXINPUT7_PIN
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
#endif

#endif // SDCARD_ENABLE || ETHERNET_ENABLE

#if I2C_ENABLE
#define I2C_PORT                1
#define I2C_SDA                 6    
#define I2C_SCL                 7
#endif

#if MODBUS_ENABLE
#define MODBUS_RTU_STREAM          0
#define UART_RX_PIN          29
#define UART_TX_PIN          28
#endif
