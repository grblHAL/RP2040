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

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "PicoBOB Deluxe G540"
#define BOARD_URL "https://github.com/Expatria-Technologies/PicoBOB_DLX"

#ifdef PICO_FLASH_SIZE_BYTES
#undef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

#define LITTLEFS_ENABLE 1

// Define step pulse output pins.
#define STEP_PORT             GPIO_PIO  // N_AXIS pin PIO SM
#define STEP_PINS_BASE        22        // N_AXIS number of consecutive pins are used by PIO

// Define step direction output pins.
#define DIRECTION_PORT        GPIO_OUTPUT
#define DIRECTION_OUTMODE     GPIO_MAP
#define X_DIRECTION_PIN       9
#define Y_DIRECTION_PIN       10
#define Z_DIRECTION_PIN       11

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
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

// Define homing/hard limit switch input pins.  Currently configured so that X, Y and Z limit pins are shared.
#define LIMIT_PORT            GPIO_INPUT
#define X_LIMIT_PIN           2
#define Y_LIMIT_PIN           1
#define Z_LIMIT_PIN           2

// Define Aux Outputs
#define AUXOUTPUT0_PORT       GPIO_OUTPUT // GPIO LED
#define AUXOUTPUT0_PIN        7
#define AUXOUTPUT1_PORT       GPIO_OUTPUT // Spindle PWM
#define AUXOUTPUT1_PIN        14
#define AUXOUTPUT2_PORT       GPIO_OUTPUT // Spindle enable
#define AUXOUTPUT2_PIN        13
#define AUXOUTPUT3_PORT       GPIO_OUTPUT // Coolant flood
#define AUXOUTPUT3_PIN        8

// Define driver spindle pins
// No direction signal on the Mach3 BOB.
// Spindle relay control is shared with B direction port, only one can be enabled at a time!
#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_PORT            GPIO_OUTPUT
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT1_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif

//Stepper enable is replaced with coolant control
#if COOLANT_ENABLE
#define COOLANT_PORT            GPIO_OUTPUT
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#undef COOLANT_ENABLE
#ifdef COOLANT_FLOOD_PIN
#define COOLANT_ENABLE COOLANT_FLOOD
#else
#define COOLANT_ENABLE 0
#endif
#endif

// Define Aux inputs
#define AUXINPUT0_PIN         6
#define AUXINPUT1_PIN         15
#define AUXINPUT3_PIN         27 // I2C strobe
#define AUXINPUT4_PIN         4  // Probe
#ifndef I2C_STROBE_ENABLE
  #define AUXINPUT5_PIN       3  // Reset
#else
  //reset pin is swapped to keypad when present, frees up additional input.
  #define AUXINPUT5_PIN       15 // Reset
  #define AUXINPUT2_PIN       3
#endif

#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN             AUXINPUT5_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN             AUXINPUT4_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PIN        AUXINPUT3_PIN
#endif

#if I2C_ENABLE
#define I2C_PORT              0
#define I2C_SDA               28
#define I2C_SCL               29
#endif

#if ETHERNET_ENABLE
#define SPI_PORT              0
#define SPI_SCK_PIN           18
#define SPI_MOSI_PIN          19
#define SPI_MISO_PIN          16
#define SPI_CS_PIN            17
#define SPI_IRQ_PIN           20
#define SPI_RST_PORT          GPIO_OUTPUT
//#define SPI_RST_PIN         21
#endif
