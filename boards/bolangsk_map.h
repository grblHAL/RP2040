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

// Pin assignments corrected to match the factory pin definition file for this
// board (schematic SCH_ENC_PI_2 "core" sheet, 2026-08-15) and the pin map
// reported by the factory pre-installed firmware ($pins output).
//
// HARDWARE VERIFIED (2026-11-14, bench test with serial monitor):
// every control/probe/limit chain was exercised end-to-end by grounding the
// corresponding input terminal and checking the reported Pn: letter and/or
// probe report in grblHAL.
//
// IMPORTANT - board silkscreen defect: on the bottom input terminal row the
// printed labels ESTOP/HOLD/START/RESET/PROBE/IN1/IN2/IN3 are SHIFTED 3
// terminal positions relative to the actual PCB routing. Verified positions
// (counted from the RIGHT edge of the row):
//
//   pos 1-3  (silk ESTOP/HOLD/START) : not connected, no firmware reaction
//   pos 4    (silk RESET)            : E-STOP input   (GPIO 35) - Pn:E + Alarm:10
//   pos 5    (silk PROBE)            : FEED HOLD      (GPIO 34) - Pn:H
//   pos 6    (silk IN1)              : CYCLE START    (GPIO 33) - Pn:S
//   pos 7    (silk IN2)              : on-board reset button (GPIO 32), unbound
//   pos 8    (silk IN3)              : PROBE          (GPIO 31) - Pn:P + live
//                                      [PRB:] during a G38.2 cycle
//
// The label shift is limited to this control block: further along the row the
// silkscreen is correct (verified: "B" terminal -> limit B input (Pn:B),
// "LIMIT A" -> limit A (Pn:A), "AXIS Z" -> limit Z (Pn:Z), "Y" -> limit Y
// (Pn:Y), "X" -> limit X (Pn:X)).
//
// Electrical notes: the control, probe and limit input terminals are fed from
// +24V through on-board optocouplers; an input is ACTIVE when pulled to GND
// (NPN/relay contact, sinking). Measured with a DMM (terminal vs GND, open):
// ~23-24V. Trigger polarity settings used during verification: $14=70,
// $6=1 (probe), $5=31 (limit invert... see grblHAL settings docs).
//
// The PULSE and SPINDLE (encoder) terminals are a DIFFERENT input type:
// measured ~1.6V open (logic-side input stage, no 24V optocoupler). These
// expect an encoder signal with open-collector/3.3-5V levels - do NOT wire
// 24V devices to them.
//
// NOTE: the factory schematic net names for the control inputs are
// JITING (E-stop), ZANTING (feed hold), QIDO (cycle start), TANCE (probe) and
// describe the REAL routing, not the printed silkscreen labels.

#if RP_MCU != 2350
#error "Board has a RP2350 processor!"
#endif

#define BOARD_NAME "Bolangsk"

// Primary UART0: TX 0 / RX 1 - wired to the LCD connector on the board
// (micro JST GH 1.25, pinout: GND, RX, TX, +5V).
// Suitable for a SmartPendant (MPG & DRO mode) with USB as the primary stream.
// Dedicated MPG handwheel input: GPIO 27, RX only, RS232 levels, isolated.

#undef SERIAL1_PORT
#define SERIAL1_PORT            0
#define UART_1_RX_PIN           1
#define UART_1_TX_PIN           0
#undef SERIAL2_PORT
#define SERIAL2_PORT            1 // PIO based, RX only
#define UART_2_RX_PIN           27

#if MODBUS_ENABLE && (MODBUS_ENABLE & MODBUS_RTU_ENABLED)
#define MODBUS_RTU_STREAM 1
#endif

#if MPG_ENABLE && !defined(MPG_STREAM)
#define MPG_STREAM              2 // RX-only GPIO 27 input, for a serial MPG handwheel.
                                  // Set to 1 to use UART0 (LCD connector) for a
                                  // SmartPendant instead.
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
#define AUXOUTPUT0_PIN          37
#define AUXOUTPUT1_PORT         GPIO_OUTPUT  //out1
#define AUXOUTPUT1_PIN          38
#define AUXOUTPUT2_PORT         GPIO_OUTPUT  //out2
#define AUXOUTPUT2_PIN          39
#define AUXOUTPUT3_PORT         GPIO_OUTPUT  //out3
#define AUXOUTPUT3_PIN          40
#define AUXOUTPUT4_PORT         GPIO_OUTPUT  //out4
#define AUXOUTPUT4_PIN          41
#define AUXOUTPUT5_PORT         GPIO_OUTPUT  //out5
#define AUXOUTPUT5_PIN          42
#define AUXOUTPUT6_PORT         GPIO_OUTPUT  // Spindle enable
#define AUXOUTPUT6_PIN          46
#define AUXOUTPUT7_PORT         GPIO_OUTPUT  // Spindle PWM
#define AUXOUTPUT7_PIN          22
#define AUXOUTPUT8_PORT         GPIO_OUTPUT  // Spindle direction
#define AUXOUTPUT8_PIN          45
#define AUXOUTPUT9_PORT         GPIO_OUTPUT  // Coolant flood
#define AUXOUTPUT9_PIN          43
#define AUXOUTPUT10_PORT        GPIO_OUTPUT  // Coolant mist
#define AUXOUTPUT10_PIN         44

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_PORT            GPIO_OUTPUT
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT6_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT7_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT8_PIN
#endif

#if COOLANT_ENABLE
#define COOLANT_PORT            GPIO_OUTPUT
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT9_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT10_PIN
#endif

#define AUXINPUT0_PIN           36 //in0
#define AUXINPUT1_PIN           11 //in1
#define AUXINPUT2_PIN           10 //in2
#define AUXINPUT3_PIN           9  //in3
#define AUXINPUT4_PIN           8  //in4
#define AUXINPUT5_PIN           7  //in5
#define AUXINPUT6_PIN           6  //in6
#if SPINDLE_ENCODER_ENABLE
#define SPINDLE_PULSE_PIN       29  // Must be an odd pin
#define SPINDLE_INDEX_PIN       28
#else
#define AUXINPUT7_PIN           29 //Spindle|pulse pin on the board
#define AUXINPUT8_PIN           28 //Spindle|index pin on the board
#endif
#define AUXINPUT9_PIN           35 // E-Stop (board label: JITING)
#define AUXINPUT10_PIN          34 // Feed hold (board label: ZANTING)
#define AUXINPUT11_PIN          33 // Cycle start (board label: QIDO)
#define AUXINPUT12_PIN          31 // Probe (board label: TANCE)

// Define user-control controls (cycle start, reset, feed hold) input pins.
// NOTE: the core does not have a separate E-stop input, the halt/reset input
// is used for the E-stop button (GPIO 35). The dedicated reset button
// (GPIO 32) can be wired in parallel with it or left unused.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN               AUXINPUT9_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PIN           AUXINPUT10_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN         AUXINPUT11_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN               AUXINPUT12_PIN
#endif

#if SDCARD_ENABLE
#define SPI_PORT                0
#define SPI_SCK_PIN             2
#define SPI_MOSI_PIN            3 // SD card CMD net
#define SPI_MISO_PIN            4 // SD card DATA0 net
#define SD_CS_PIN               5
#endif // SDCARD_ENABLE

// NOTE: no I2C peripherals are wired to dedicated pins on this board;
// GPIO 8/9 are general purpose input terminals (IN4/IN3).
