/*
  Sienci Labs SLB-Lite driver pin map

  Part of grblHAL

  Copyright (c) 2026 - Sienci Labs Inc.

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
  #error "Board has a RP2350B 80-Pin processor! - Make sure to use Board: pimoroni_pga2350 in VSCode"
#endif

#define BOARD_NAME "SLB Lite"
#define HAS_BOARD_INIT

#define AUXOUTPUT0_PORT         GPIO_OUTPUT // Homing XYA
#define AUXOUTPUT0_PIN          24
#define AUXOUTPUT1_PORT         GPIO_OUTPUT // Homing Z
#define AUXOUTPUT1_PIN          25
#define AUXOUTPUT2_PORT         GPIO_OUTPUT // Spindle PWM
#define AUXOUTPUT2_PIN          40
#define AUXOUTPUT3_PORT         GPIO_OUTPUT // Laser PWM
#define AUXOUTPUT3_PIN          29
#define AUXOUTPUT4_PORT         GPIO_OUTPUT // Laser Enable
#define AUXOUTPUT4_PIN          28
#define AUXOUTPUT5_PORT         GPIO_OUTPUT // Modbus dir
#define AUXOUTPUT5_PIN          38
#define AUXOUTPUT6_PORT         GPIO_OUTPUT // Unused Spare Output on Expansion Header
#define AUXOUTPUT6_PIN          26
#define AUXOUTPUT7_PORT         GPIO_OUTPUT // Unused Spare Output on Expansion Header
#define AUXOUTPUT7_PIN          47

#define HOME_INDICATOR_XYZA_PIN AUXOUTPUT0_PIN
#define HOME_INDICATOR_Z_PIN    AUXOUTPUT1_PIN

#define AUXINPUT0_PIN           11 // Motor fault M4
#define AUXINPUT1_PIN           10 // Motor fault M3
#define AUXINPUT2_PIN            7 // Motor fault X
#define AUXINPUT3_PIN            8 // Motor fault Y
#define AUXINPUT4_PIN            9 // Motor fault Z
#define AUXINPUT5_PIN           23 // Toolsetter
#define AUXINPUT6_PIN           22 // Probe
#define AUXINPUT7_PIN           39 // Reset/EStop
#define AUXINPUT8_PIN           27 // Spindle Detect

#define USE_EXPANDERS // Uses 74HCT595 Expansion on PIO
#define OUT_SHIFT_REGISTER       8 // How many bits are used in the shift register, 8 for 74HCT595
#define OUT_SR_DATA_PIN         32 // Data Pin
#define OUT_SR_SCK_PIN          33 // Clock Pin (includes next pin (34) automatically as Latch)
#define OUT_SR_OE_PIN           26 // Output Enable (active low)

// Define step pulse output pins: PIO Peripheral: Define Base Pin and then consecutive pins for the number of axes are assigned automatically
#define STEP_PORT               GPIO_PIO  // N_AXIS pin PIO SM
#define STEP_PINS_BASE          17        // N_AXIS number of consecutive pins are used by PIO

// Define step direction output pins: Standard GPIO
#define DIRECTION_PORT          GPIO_OUTPUT
#define X_DIRECTION_PIN         12
#define Y_DIRECTION_PIN         13
#define Z_DIRECTION_PIN         14
#define DIRECTION_OUTMODE       GPIO_SHIFT12

// Define stepper driver enable/disable output pin.
#define XY_ENABLE_PORT          EXPANDER_PORT
#define XY_ENABLE_PIN           0
#define Z_ENABLE_PORT           EXPANDER_PORT
#define Z_ENABLE_PIN            1

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN             6
#define Y_LIMIT_PIN             5
#define Z_LIMIT_PIN             4
#define LIMIT_INMODE            GPIO_MAP

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_DIRECTION_PIN        15
#define M3_LIMIT_PIN            3
#endif

#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_DIRECTION_PIN        16
#define M4_LIMIT_PIN            2
#endif

// Modbus RTU: Spindle/VFD
#define SERIAL1_PORT  1 // Modbus RTU
#define SERIAL1_PORT_PIO // Use the PIO-backed UART implementation for SERIAL1_PORT.
#ifdef SERIAL1_PORT
  #define UART_1_RX_PIN           36
  #define UART_1_TX_PIN           37
#endif
#if MODBUS_ENABLE
  #define MODBUS_RTU_STREAM       1
  #undef MODBUS_ENABLE
  #define MODBUS_ENABLE           (MODBUS_RTU_ENABLED|MODBUS_RTU_DIR_ENABLED)
  #define MODBUS_DIR_AUX          5 // D38
#endif

// PWM: Spindle Port
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
  #define SPINDLE_PWM_PORT        GPIO_OUTPUT
  #define SPINDLE_PWM_PIN         AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
  #define SPINDLE_ENABLE_PORT   EXPANDER_PORT
  #define SPINDLE_ENABLE_PIN    3 // Bit 3 on shift register
#endif

// PWM2: Laser Port
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
  #define SPINDLE1_PWM_PORT       GPIO_OUTPUT
  #define SPINDLE1_PWM_PIN        AUXOUTPUT3_PIN
#endif
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_ENA
  #define SPINDLE1_ENABLE_PORT   GPIO_OUTPUT
  #define SPINDLE1_ENABLE_PIN    AUXOUTPUT4_PIN
#endif

// Coolant port managed from Event Plugin instead, so disabled here
#if COOLANT_ENABLE
  #undef COOLANT_ENABLE
  #define COOLANT_ENABLE 0
#endif

// Onboard and Offboard Neopixels
#if RGB_LED_ENABLE
  #define NEOPIXELS_PIN           35
#endif

// Emergency Stop Input
#if CONTROL_ENABLE
  #undef CONTROL_ENABLE
  #define CONTROL_ENABLE CONTROL_HALT // EStop is treated as a hard control input that halts the machine and requires a reset to clear
  #define RESET_PIN               AUXINPUT7_PIN
#endif

// Probe and Tool Length Sensors
#if PROBE_ENABLE
  #define PROBE_PIN               AUXINPUT6_PIN
#endif
#if TOOLSETTER_ENABLE
  #define TOOLSETTER_PIN          AUXINPUT5_PIN
#endif

// CLS Motor Fault/Alarm Inputs
#if MOTOR_FAULT_ENABLE
  #define X_MOTOR_FAULT_PIN       AUXINPUT2_PIN
  #define Y_MOTOR_FAULT_PIN       AUXINPUT3_PIN
  #define Z_MOTOR_FAULT_PIN       AUXINPUT4_PIN
  // Satisfy the generic single-pin motor-fault sanity check.
  // SLB Lite overrides control-state handling in board_init() to aggregate per-axis faults.
  #define MOTOR_FAULT_PIN         X_MOTOR_FAULT_PIN
  #ifdef M3_AVAILABLE
    #define M3_MOTOR_FAULT_PIN      AUXINPUT1_PIN
  #endif
  #ifdef M4_AVAILABLE
    #define M4_MOTOR_FAULT_PIN      AUXINPUT0_PIN
  #endif
#endif

// SPI: SD and Ethernet
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
    #define WIZNET_CS_PIN           SPI_CS_PIN
  #endif
#endif // SDCARD_ENABLE || ETHERNET_ENABLE

// I2C: ATC Expansion
#if I2C_ENABLE
  #define I2C_PORT                1
  #define I2C_SDA                 30
  #define I2C_SCL                 31
#endif

// End of Pinmap for Sienci Labs SLB-Lite
