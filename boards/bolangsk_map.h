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

// Pin assignments for the Bolangsk 5-axis controller (RP2350B).
//
// CONTROL INPUT MAP - HARDWARE VERIFIED (2026-11-14, live bench test): each
// control terminal in the bottom input row was grounded through its on-board
// optocoupler and the firmware reaction (Pn: letter / alarm) checked against
// $pins. The PCB routes the row in DESCENDING GPIO order from the RIGHT edge,
// one-to-one with the silkscreen labels:
//
//   silk ESTOP -> GPIO 38   (E-stop / halt input) : Pn:E + Alarm:10
//   silk HOLD  -> GPIO 37   (feed hold)           : Pn:H
//   silk START -> GPIO 36   (cycle start)         : Pn:S
//   silk RESET -> GPIO 35   (spare input, AUXINPUT0)
//   silk PROBE -> GPIO 34   (probe)               : Pn:P + live [PRB:]
//   silk IN1   -> GPIO 33   (spare input, AUXINPUT1)
//   silk IN2   -> GPIO 32   (spare input, AUXINPUT2; also on-board reset button)
//   silk IN3   -> GPIO 31   (spare input, AUXINPUT3)
//
// Direct evidence (gathered on an earlier schematic-derived build whose
// control block was shifted +3 positions): grounding the PROBE terminal fired
// feed hold (GPIO 34 -> Pn:H), the RESET terminal fired E-stop (35 ->
// Alarm:10), IN1 fired cycle start (33 -> Pn:S) and IN3 fired the probe
// (31 -> Pn:P) - proving the terminals are physically wired to 34/35/33/31.
// The factory WebBuilder pin map (E=38, H=37, S=36, P=34) matches this
// routing; the control input block below follows it.
//
// CAUTION: the factory schematic (SCH_ENC_PI, MCU sheet) places
// EStop_uC/Hold_uC/CycleStart_uC/Probe_uC on GPIOs 35/34/33/31 - a +3
// position shift relative to the real PCB routing. Do not trust the
// schematic connector pages for the control block; the silkscreen is right.
//
// Further along the row the silkscreen is correct (verified): "B" -> limit B
// (Pn:B), "LIMIT A" -> limit A (Pn:A), "AXIS Z" -> limit Z (Pn:Z), "Y" ->
// limit Y (Pn:Y), "X" -> limit X (Pn:X).
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
// NOTE: GPIOs 11/10/9 (former AUXINPUT1-3 assignments) are left unbound -
// their physical terminal locations have not been verified. AUXINPUT4-6
// (GPIO 8/7/6) kept as-is; locations unverified.

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

#define AUXINPUT0_PIN           35 //in0 - RESET terminal (spare control input)
#define AUXINPUT1_PIN           33 //in1 - IN1 terminal
#define AUXINPUT2_PIN           32 //in2 - IN2 terminal (also on-board reset button)
#define AUXINPUT3_PIN           31 //in3 - IN3 terminal
#define AUXINPUT4_PIN           8  //in4 - location unverified
#define AUXINPUT5_PIN           7  //in5 - location unverified
#define AUXINPUT6_PIN           6  //in6 - location unverified
#if SPINDLE_ENCODER_ENABLE
#define SPINDLE_PULSE_PIN       29  // Must be an odd pin
#define SPINDLE_INDEX_PIN       28
#else
#define AUXINPUT7_PIN           29 //Spindle|pulse pin on the board
#define AUXINPUT8_PIN           28 //Spindle|index pin on the board
#endif
#define AUXINPUT9_PIN           38 // E-Stop (silk: ESTOP, rightmost terminal)
#define AUXINPUT10_PIN          37 // Feed hold (silk: HOLD)
#define AUXINPUT11_PIN          36 // Cycle start (silk: START)
#define AUXINPUT12_PIN          34 // Probe (silk: PROBE)

// Define user-control controls (cycle start, reset, feed hold) input pins.
// NOTE: the core does not have a separate E-stop input, the halt/reset input
// is used for the E-stop button (GPIO 38). The RESET terminal (GPIO 35) and
// IN2 (GPIO 32, also wired to the on-board reset button) are available as
// spare aux inputs.
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

// NOTE: no I2C peripherals are wired to dedicated pins on this board.
// GPIOs 11/10/9 (former AUXINPUT1-3) are unbound; physical terminal
// locations not verified.
