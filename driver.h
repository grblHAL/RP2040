/*

  driver.h - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

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

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <stdbool.h>
#include <stdint.h>

#include "hardware/pio.h"
#include "hardware/gpio.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#if SDCARD_ENABLE
#define SPI_ENABLE 1
#endif

#if ETHERNET_ENABLE
#ifndef _WIZCHIP_
#define _WIZCHIP_ 5105 // W5100S
#endif
#ifndef SPI_ENABLE
#define SPI_ENABLE 1
#endif
#endif

#if defined(WEBUI_ENABLE) && WEBUI_ENABLE
#ifndef WEBUI_INFLASH
#define WEBUI_INFLASH 1
#endif
#if WEBUI_INFLASH
#if defined(LITTLEFS_ENABLE) && LITTLEFS_ENABLE == 0
#undef LITTLEFS_ENABLE
#endif
#ifndef LITTLEFS_ENABLE
#define LITTLEFS_ENABLE 1
#endif
#endif
#endif

#include "grbl/driver_opts.h"

#if ETHERNET_ENABLE && WIFI_ENABLE
#error "WiFi and Ethernet cannot be enabled at the same time!"
#endif

#if NUM_BANK0_GPIOS <= 32
#define DIGITAL_IN(pin) (!!(gpio_get_all() & (1UL << (pin))))
#define DIGITAL_OUT(pin, on) gpio_put(pin, on)
#else
#define DIGITAL_IN(pin) (!!(gpio_get_all64() & (1ULL << (pin))))
#define DIGITAL_OUT(pin, on) gpio_put(pin, on)
#endif

#define EXPANDER_IN(pin) (iox_out[pin] && iox_out[pin]->get_value(iox_out[pin]) != 0.0f)
#define EXPANDER_OUT(pin, state) { if(iox_out[pin]) iox_out[pin]->set_value(iox_out[pin], (float)(state)); }


#define GPIO_IRQ_ALL (GPIO_IRQ_LEVEL_HIGH|GPIO_IRQ_LEVEL_LOW|GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL)

// Define GPIO mode options

#define GPIO_SHIFT0    0
#define GPIO_SHIFT1    1
#define GPIO_SHIFT2    2
#define GPIO_SHIFT3    3
#define GPIO_SHIFT4    4
#define GPIO_SHIFT5    5
#define GPIO_SHIFT6    6
#define GPIO_SHIFT7    7
#define GPIO_SHIFT8    8
#define GPIO_SHIFT9    9
#define GPIO_SHIFT10  10
#define GPIO_SHIFT11  11
#define GPIO_SHIFT12  12
#define GPIO_SHIFT13  13
#define GPIO_SHIFT14  14
#define GPIO_SHIFT15  15
#define GPIO_SHIFT16  16
#define GPIO_SHIFT17  17
#define GPIO_SHIFT18  18
#define GPIO_SHIFT19  19
#define GPIO_SHIFT20  20
#define GPIO_SHIFT21  21
#define GPIO_SHIFT22  22
#define GPIO_SHIFT23  23
#define GPIO_SHIFT24  24
#define GPIO_SHIFT25  25
#define GPIO_SHIFT26  26
#define GPIO_SHIFT27  27
#define GPIO_SHIFT28  28
#define GPIO_MAP      31
#define GPIO_DIRECT   33
#define GPIO_IOEXPAND 34
#define GPIO_INPUT    35
#define GPIO_OUTPUT   36
#define GPIO_PIO      37
#define GPIO_PIO_1    38
#define GPIO_SR8      39
#define GPIO_SR16     40

#define STEPPERS_ENABLE_PINMODE 0

// Define timer allocations.

/*
#define RPM_COUNTER_N               3
#define RPM_COUNTER                 timer(RPM_COUNTER_N)
#define RPM_COUNTER_IRQn            timerINT(RPM_COUNTER_N)
#define RPM_COUNTER_IRQHandler      timerHANDLER(RPM_COUNTER_N)

#define RPM_TIMER_N                 2
#define RPM_TIMER                   timer(RPM_TIMER_N)
#define RPM_TIMER_IRQn              timerINT(RPM_TIMER_N)
#define RPM_TIMER_IRQHandler        timerHANDLER(RPM_TIMER_N)

#define PPI_TIMER_N                 2
#define PPI_TIMER                   timer(PPI_TIMER_N)
#define PPI_TIMER_IRQn              timerINT(PPI_TIMER_N)
#define PPI_TIMER_IRQHandler        timerHANDLER(PPI_TIMER_N)
*/

#ifndef SERIAL_PORT
#define SERIAL_PORT 0
#endif

#ifndef CONTROL_ENABLE
#define CONTROL_ENABLE (CONTROL_HALT|CONTROL_FEED_HOLD|CONTROL_CYCLE_START)
#endif

#ifdef BOARD_CNC_BOOSTERPACK
  #include "boards/cnc_boosterpack_map.h"
#elif defined(BOARD_PICO_CNC)
  #include "boards/pico_cnc_map.h"
#elif defined(BOARD_RP23U5XBB)
  #include "boards/RP2350B_5X_map.h"
#elif defined(BOARD_PICOBOB)
  #include "boards/picobob_map.h"
#elif defined(BOARD_PICOBOB_G540)
  #include "boards/picobob_g540_map.h"  
#elif defined(BOARD_PICOBOB_DLX)
  #include "boards/picobob_dlx_map.h" 
#elif defined(BOARD_PICOBOB_DLX_G540)
  #include "boards/picobob_dlx_g540_map.h"
#elif defined(BOARD_PICOHAL)
  #include "boards/picohal_map.h"        
#elif defined(BOARD_BTT_SKR_PICO_10)
  #include "boards/btt_skr_pico_10_map.h"
#elif defined BOARD_CITOH_CX6000
  #include "boards/citoh_cx6000_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "boards/my_machine_map.h"
#elif defined(BOARD_GENERIC_4AXIS)
  #include "boards/generic_map_4axis.h"
#elif defined(BOARD_GENERIC_8AXIS)
  #include "boards/generic_map_8axis.h"
#else // default board
  #include "boards/generic_map.h"
#endif

#ifndef STEP_PULSE_TOFF_MIN
#define STEP_PULSE_TOFF_MIN 2.0f
#endif

#if SPI_ENABLE && !defined(SPI_DMA_ENABLE)
#define SPI_DMA_ENABLE 1
#endif

#if STEP_PORT == GPIO_PIO
#define X_STEP_PIN STEP_PINS_BASE + 0
#define Y_STEP_PIN STEP_PINS_BASE + 1
#define Z_STEP_PIN STEP_PINS_BASE + 2
#if defined(M3_AVAILABLE) && !defined(M3_STEP_PIN)
#define M3_STEP_PIN STEP_PINS_BASE + 3
#endif
#if defined(M4_AVAILABLE) && !defined(M4_STEP_PIN)
#define M4_STEP_PIN STEP_PINS_BASE + 4
#endif
#if defined(M5_AVAILABLE) && !defined(M5_STEP_PIN)
#define M5_STEP_PIN STEP_PINS_BASE + 5
#endif
#if defined(M6_AVAILABLE) && !defined(M6_STEP_PIN)
#define M6_STEP_PIN STEP_PINS_BASE + 6
#endif
#if defined(M7_AVAILABLE) && !defined(M7_STEP_PIN)
#define M7_STEP_PIN STEP_PINS_BASE + 7
#endif
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.0f // microseconds
#endif

// End configuration

#include "grbl/driver_opts2.h"

#if I2C_ENABLE && !defined(I2C_PORT)
#define I2C_PORT    1
#define I2C_SDA     26
#define I2C_SCL     27
#endif

// End configuration

#if SDCARD_ENABLE && !defined(SD_CS_PIN)
#error SD card plugin not supported!
#endif

#ifndef STEP_PINMODE
#define STEP_PINMODE PINMODE_OUTPUT
#endif

#ifndef DIRECTION_PINMODE
#define DIRECTION_PINMODE PINMODE_OUTPUT
#endif

#ifndef STEPPERS_DISABLE_PINMODE
#define STEPPERS_DISABLE_PINMODE PINMODE_OUTPUT
#endif

typedef struct {
    pin_function_t id;
    pin_cap_t cap;
    pin_mode_t mode;
    uint8_t pin;
    uint8_t user_port;
    pin_group_t group;
    uint32_t port;
    volatile bool active;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    pin_group_t group;
    uint8_t pin;
    uint32_t port;
    pin_mode_t mode;
    const char *description;
    uint8_t pwm_idx;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

bool driver_init (void);

#if OUT_SHIFT_REGISTER
typedef struct {
    PIO pio;
    uint sm;
} sr_reg_t;
#endif

void board_init (void);

#if SPI_RST_PORT == EXPANDER_PORT
void spi_reset_out (bool on);
#endif

void ioports_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);
void pinEnableIRQ (const input_signal_t *input, pin_irq_mode_t irq_mode);

/**
  \brief   Enable IRQ Interrupts
  \details Enables IRQ interrupts by clearing the I-bit in the CPSR.
           Can only be executed in Privileged modes.
 */

// While waiting for CMSIS headers...:

static inline void __enable_irq(void)
{
  __asm volatile ("cpsie i" : : : "memory");
}

/**
  \brief   Disable IRQ Interrupts
  \details Disables IRQ interrupts by setting the I-bit in the CPSR.
           Can only be executed in Privileged modes.
 */
static inline  void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}

#endif // __DRIVER_H__
