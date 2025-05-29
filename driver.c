/*

  driver.c - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021 Volksolive
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

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <malloc.h>

#include "pico/time.h"
#include "pico/bootrom.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/sio.h"
#if RP_MCU == 2040
#include "hardware/rtc.h"
#if defined(PICO_USE_FASTEST_SUPPORTED_CLOCK) && PICO_USE_FASTEST_SUPPORTED_CLOCK
#define PIO_STEP_ADJ 0.2f
#else
#define PIO_STEP_ADJ 0.29f
#endif
#define PIO_RATE_ADJ 16
#else
#define PIO_STEP_ADJ 0.2f
#define PIO_RATE_ADJ 14
#endif

#include "driver.h"
#include "serial.h"
#include "driverPIO.pio.h"
#include "ws2812.pio.h"

//#define AUX_DEVICES // until all drivers are converted?  **AM Could not get the board to properly enumerate the ports without removing this.
#ifndef AUX_CONTROLS
#define AUX_CONTROLS (AUX_CONTROL_SPINDLE|AUX_CONTROL_COOLANT)
#endif

#include "grbl/crossbar.h"
#include "grbl/machine_limits.h"
#include "grbl/state_machine.h"
#include "grbl/motor_pins.h"
#include "grbl/pin_bits_masks.h"
#include "grbl/protocol.h"
#if NVSDATA_BUFFER_ENABLE
#include "grbl/nvs_buffer.h"
#endif

#ifdef I2C_PORT
#include "i2c.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "ff.h"
#include "diskio.h"
#endif

#if LITTLEFS_ENABLE
#include "littlefs_hal.h"
#include "sdcard/fs_littlefs.h"
#include "sdcard/macros.h"
#endif

#if USB_SERIAL_CDC
#include "usb_serial.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if PPI_ENABLE
#include "laser/ppi.h"
#endif

#if FLASH_ENABLE
#include "flash.h"
#endif

#if WIFI_ENABLE
#include "wifi.h"
#endif

#if BLUETOOTH_ENABLE == 1
#include "bt_native.h"
#endif

#if WIFI_ENABLE || BLUETOOTH_ENABLE == 1
#include "pico/cyw43_arch.h"
#endif

#if ETHERNET_ENABLE
#include "networking/wiznet/enet.h"
#endif

#if STEP_PORT == GPIO_SR8
static PIO sr8_pio;
static uint sr8_sm;
static PIO sr8_delay_pio;
static uint sr8_delay_sm;
static PIO sr8_hold_pio;
static uint sr8_hold_sm;
#elif STEP_PORT == GPIO_PIO
static PIO step_pio;
static uint step_sm;
#elif STEP_PORT == GPIO_PIO_1
static PIO x_step_pio;
static uint x_step_sm;
static PIO y_step_pio;
static uint y_step_sm;
static PIO z_step_pio;
static uint z_step_sm;
#ifdef X2_STEP_PIN
static PIO x2_step_pio;
static uint x2_step_sm;
#endif
#ifdef Y2_STEP_PIN
static PIO y2_step_pio;
static uint y2_step_sm;
#endif
#ifdef Z2_STEP_PIN
static PIO z2_step_pio;
static uint z2_step_sm;
#endif
#ifdef A_STEP_PIN
static PIO a_step_pio;
static uint a_step_sm;
#endif
#ifdef B_STEP_PIN
static PIO b_step_pio;
static uint b_step_sm;
#endif
#ifdef C_STEP_PIN
static PIO c_step_pio;
static uint c_step_sm;
#endif
#endif // GPIO_PIO_1

#ifdef NEOPIXELS_PIN

#ifndef NEOPIXELS_NUM
#define NEOPIXELS_NUM 0
#endif

static PIO neop_pio;
static uint neop_sm;

#endif

typedef union {
    uint32_t value;
    struct {
        uint32_t delay  :8,
                 length :8,
                 set    :8,
                 reset  :8;
    };
} pio_steps_t;

#if ETHERNET_ENABLE || WIFI_ENABLE || BLUETOOTH_ENABLE == 1

typedef union {
    uint8_t value;
    struct {
        uint8_t wifi      :1,
                bluetooth :1,
                ethernet  :1,
                unused    :5;
    };
} net_types_t;

static net_types_t net = {0};

#endif

static float pio_clk;
#if DRIVER_SPINDLE_ENABLE
static spindle_id_t spindle_id = -1;
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
static bool pwmEnabled = false;
static spindle_pwm_t spindle_pwm;
#endif

static pio_steps_t pio_steps = {.delay = 20, .length = 100};
static uint stepper_timer_sm, stepper_timer_sm_offset;
static bool IOInitDone = false;
static status_code_t (*on_unknown_sys_command)(uint_fast16_t state, char *line, char *lcline);
static volatile uint32_t elapsed_ticks = 0;
static probe_state_t probe = { .connected = On };
static pin_group_pins_t limit_inputs;
#if SAFETY_DOOR_BIT
static input_signal_t *safety_door;
#endif

#ifdef USE_EXPANDERS
static xbar_t *iox_out[32] = {};
#endif

#ifdef NEOPIXELS_PIN
neopixel_cfg_t neopixel = { .intensity = 255 };
#endif

#if SPINDLE_ENCODER_ENABLE

#include "grbl/spindle_sync.h"

#define RPM_TIMER_RESOLUTION 1

static uint encoder_pwm;
static uint32_t encoder_ovf;
static absolute_time_t encoder_started;
static spindle_data_t spindle_data;
static spindle_encoder_t spindle_encoder = {
    .tics_per_irq = 4
};
static on_spindle_programmed_ptr on_spindle_programmed = NULL;

#endif // SPINDLE_ENCODER_ENABLE

#include "grbl/stepdir_map.h"

static input_signal_t *irq_pins[NUM_BANK0_GPIOS] = {};
static periph_signal_t *periph_pins = NULL;

#define add_in_pin(p) defined(p##_PIN)
#define add_limit_pin(p) { .id = Input_Limit##p, .port = GPIO_INPUT, .pin = p##_LIMIT_PIN, .group = PinGroup_Limit },
#define add_maxlimit_pin(p) { .id = Input_Limit##p##_Max, .port = GPIO_INPUT, .pin = p##_LIMIT_PIN_MAX, .group = PinGroup_Limit },

static input_signal_t inputpin[] = {
#ifdef RESET_PIN
#if ESTOP_ENABLE
    { .id = Input_EStop, .port = GPIO_INPUT, .pin = RESET_PIN, .group = PinGroup_Control },
#else
    { .id = Input_Reset, .port = GPIO_INPUT, .pin = RESET_PIN, .group = PinGroup_Control },
#endif
#endif
#ifdef FEED_HOLD_PIN
    { .id = Input_FeedHold, .port = GPIO_INPUT, .pin = FEED_HOLD_PIN, .group = PinGroup_Control },
#endif
#ifdef CYCLE_START_PIN
    { .id = Input_CycleStart, .port = GPIO_INPUT, .pin = CYCLE_START_PIN, .group = PinGroup_Control },
#endif
#if SAFETY_DOOR_BIT
    { .id = Input_SafetyDoor, .port = GPIO_INPUT, .pin = SAFETY_DOOR_PIN, .group = PinGroup_Control },
#endif
#ifdef LIMITS_OVERRIDE_PIN
    { .id = Input_LimitsOverride, .port = GPIO_INPUT, .pin = LIMITS_OVERRIDE_PIN, .group = PinGroup_Control },
#endif
    add_limit_pin(X)
    add_limit_pin(Y)
    add_limit_pin(Z)
#if add_in_pin(X2_LIMIT)
    add_limit_pin(X2)
#endif
#ifdef X_LIMIT_PIN_MAX
    add_maxlimit_pin(X)
#endif
#if add_in_pin(Y2_LIMIT)
    add_limit_pin(Y2)
#endif
#ifdef Y_LIMIT_PIN_MAX
    add_maxlimit_pin(Y)
#endif
#if add_in_pin(Z2_LIMIT)
    add_limit_pin(Z2)
#endif
#ifdef Z_LIMIT_PIN_MAX
    add_maxlimit_pin(Z)
#endif
#if add_in_pin(A_LIMIT)
    add_limit_pin(A)
#endif
#if add_in_pin(B_LIMIT)
    add_limit_pin(B)
#endif
#if add_in_pin(C_LIMIT)
    add_limit_pin(C)
#endif
#if add_in_pin(U_LIMIT)
    add_limit_pin(U)
#endif
#if add_in_pin(V_LIMIT)
    add_limit_pin(V)
#endif
#ifdef A_LIMIT_PIN_MAX
    { .id = Input_LimitA_Max,     .port = GPIO_INPUT, .pin = A_LIMIT_PIN_MAX,     .group = PinGroup_Limit },
#endif
#ifdef B_LIMIT_PIN
    { .id = Input_LimitB,         .port = GPIO_INPUT, .pin = B_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef B_LIMIT_PIN_MAX
    { .id = Input_LimitB_Max,     .port = GPIO_INPUT, .pin = B_LIMIT_PIN_MAX,     .group = PinGroup_Limit },
#endif
#ifdef C_LIMIT_PIN
    { .id = Input_LimitC,         .port = GPIO_INPUT, .pin = C_LIMIT_PIN,         .group = PinGroup_Limit },
#endif
#ifdef C_LIMIT_PIN_MAX
    { .id = Input_LimitC_Max,     .port = GPIO_INPUT, .pin = C_LIMIT_PIN_MAX,     .group = PinGroup_Limit },
#endif
#ifndef AUX_DEVICES
  #ifdef PROBE_PIN
    { .id = Input_Probe, .port = GPIO_INPUT, .pin = PROBE_PIN, .group = PinGroup_Probe },
  #endif
  #if MPG_MODE_PIN
    { .id = Input_MPGSelect, .port = GPIO_INPUT, .pin = MPG_MODE_PIN, .group = PinGroup_MPG },
  #endif
  #if I2C_STROBE_ENABLE && defined(I2C_STROBE_PIN)
    { .id = Input_I2CStrobe, .port = GPIO_INPUT, .pin = I2C_STROBE_PIN, .group = PinGroup_I2C },
  #endif
#endif // AUX_DEVICES
#ifdef SPI_IRQ_PIN
    { .id = Input_SPIIRQ,    .port = GPIO_INPUT, .pin = SPI_IRQ_PIN,    .group = PinGroup_SPI },
#endif
#ifdef SPINDLE_INDEX_PIN
    { .id = Input_SpindleIndex,   .port = GPIO_INPUT, .pin = SPINDLE_INDEX_PIN,   .group = PinGroup_SpindleIndex },
#endif
#ifdef AUXINPUT0_PIN
    { .id = Input_Aux0, .port = GPIO_INPUT, .pin = AUXINPUT0_PIN, .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT1_PIN
    { .id = Input_Aux1, .port = GPIO_INPUT, .pin = AUXINPUT1_PIN, .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT2_PIN
    { .id = Input_Aux2, .port = GPIO_INPUT, .pin = AUXINPUT2_PIN, .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT3_PIN
    { .id = Input_Aux3, .port = GPIO_INPUT, .pin = AUXINPUT3_PIN, .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT4_PIN
    { .id = Input_Aux4, .port = GPIO_INPUT, .pin = AUXINPUT4_PIN, .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT5_PIN
    { .id = Input_Aux5, .port = GPIO_INPUT, .pin = AUXINPUT5_PIN, .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT6_PIN
    { .id = Input_Aux6, .port = GPIO_INPUT, .pin = AUXINPUT6_PIN, .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT7_PIN
    { .id = Input_Aux7, .port = GPIO_INPUT, .pin = AUXINPUT7_PIN, .group = PinGroup_AuxInput }
#endif
};

#if STEP_PORT == GPIO_SR8
#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif
#define X_STEP_PIN 0
#define Y_STEP_PIN 1
#define Z_STEP_PIN 2
#ifdef X2_STEP_PORT
#undef X2_STEP_PIN
#define X2_STEP_PIN 3
#endif
#ifdef Y2_STEP_PORT
#undef Y2_STEP_PIN
#define Y2_STEP_PIN 3
#endif
#ifdef Z2_STEP_PORT
#undef Z2_STEP_PIN
#define Z2_STEP_PIN 3
#endif
#ifdef A_STEP_PORT
#undef A_STEP_PIN
#define A_STEP_PIN 3
#endif
#endif

#if DIRECTION_PORT == GPIO_SR8
#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif
#define X_DIRECTION_PIN 4
#define Y_DIRECTION_PIN 5
#define Z_DIRECTION_PIN 6
#ifdef X2_DIRECTION_PORT
#undef X2_DIRECTION_PIN
#define X2_DIRECTION_PIN 7
#endif
#ifdef Y2_DIRECTION_PORT
#undef Y2_DIRECTION_PIN
#define Y2_DIRECTION_PIN 7
#endif
#ifdef Z2_DIRECTION_PORT
#undef Z2_DIRECTION_PIN
#define Z2_DIRECTION_PIN 7
#endif
#ifdef A_DIRECTION_PORT
#undef A_DIRECTION_PIN
#define A_DIRECTION_PIN 7
#endif
#endif

#ifndef SPI_RST_PORT
#define SPI_RST_PORT GPIO_OUTPUT
#endif

#define add_aux_pin(a, p) !(AUX_CONTROLS & AUX_CONTROL_##a) && defined(a##_##p##_PIN) && a##_##p##_PORT != EXPANDER_PORT

#define add_pin(p) defined(p##_PIN) && p##_PORT != EXPANDER_PORT
#define add_step_pin(p) { .id = Output_Step##p, .port = p##_STEP_PORT, .pin = p##_STEP_PIN, .group = PinGroup_StepperStep, .mode = { STEP_PINMODE } },
#define add_dir_pin(p) { .id = Output_Dir##p, .port = p##_DIRECTION_PORT, .pin = p##_DIRECTION_PIN, .group = PinGroup_StepperStep, .mode = { DIRECTION_PINMODE } },
#define add_enable_pin(p) { .id = Output_StepperEnable##p, .port = p##_ENABLE_PORT, .pin = p##_ENABLE_PIN, .group = PinGroup_StepperEnable, .mode = { STEPPERS_ENABLE_PINMODE } },

static output_signal_t outputpin[] = {
    add_step_pin(X)
    add_step_pin(Y)
    add_step_pin(Z)
#if add_pin(X2_STEP)
    add_step_pin(X2)
#endif
#if add_pin(Y2_STEP)
    add_step_pin(Y2)
#endif
#if add_pin(Z2_STEP)
    add_step_pin(Z2)
#endif
#if add_pin(A_STEP)
    add_step_pin(A)
#endif
#if add_pin(B_STEP)
    add_step_pin(B)
#endif
#if add_pin(C_STEP)
    add_step_pin(C)
#endif
#if add_pin(U_STEP)
    add_step_pin(U)
#endif
#if add_pin(V_STEP)
    add_step_pin(V)
#endif
    add_dir_pin(X)
    add_dir_pin(Y)
    add_dir_pin(Z)
#if add_pin(X2_DIRECTION)
    add_dir_pin(X2)
#endif
#if add_pin(Y2_DIRECTION)
    add_dir_pin(Y2)
#endif
#if add_pin(Z2_DIRECTION)
    add_dir_pin(Z2)
#endif
#if add_pin(A_DIRECTION)
    add_dir_pin(A)
#endif
#if add_pin(B_DIRECTION)
    add_dir_pin(B)
#endif
#if add_pin(C_DIRECTION)
    add_dir_pin(C)
#endif
#if add_pin(U_DIRECTION)
    add_dir_pin(U)
#endif
#if add_pin(U_DIRECTION)
    add_dir_pin(V)
#endif
#if !(TRINAMIC_ENABLE && TRINAMIC_I2C)
#if add_pin(STEPPERS_ENABLE)
    { .id = Output_StepperEnable,  .port = STEPPERS_ENABLE_PORT, .pin = STEPPERS_ENABLE_PIN, .group = PinGroup_StepperEnable, .mode = { STEPPERS_ENABLE_PINMODE } },
#elif add_pin(XY_ENABLE)
    add_enable_pin(XY)
#else
#if add_pin(X_ENABLE)
    add_enable_pin(X)
#endif
#if add_pin(Y_ENABLE)
    add_enable_pin(Y)
#endif
#endif
#if add_pin(Z_ENABLE)
    add_enable_pin(Z)
#endif
#if add_pin(X2_ENABLE)
    add_enable_pin(X2)
#endif
#if add_pin(Y2_ENABLE)
    add_enable_pin(Y2)
#endif
#if add_pin(Z2_ENABLE)
    add_enable_pin(Z2)
#endif
#if add_pin(A_ENABLE)
    add_enable_pin(A)
#endif
#if add_pin(B_ENABLE)
    add_enable_pin(B)
#endif
#if add_pin(C_ENABLE)
    add_enable_pin(C)
#endif
#if add_pin(U_ENABLE)
    add_enable_pin(U)
#endif
#if add_pin(V_ENABLE)
    add_enable_pin(V)
#endif
#endif // !(TRINAMIC_ENABLE && TRINAMIC_I2C)
#if add_aux_pin(SPINDLE, PWM) || (SPINDLE_ENABLE_PORT == EXPANDER_PORT && defined(SPINDLE_PWM_PIN))
    { .id = Output_SpindlePWM,   .port = SPINDLE_PWM_PORT, .pin = SPINDLE_PWM_PIN,       .group = PinGroup_SpindlePWM },
#endif
#ifdef RTS_PIN
    { .id = Output_RTS,          .port = GPIO_OUTPUT,      .pin = RTS_PIN,               .group = PinGroup_UART },
#endif
#ifdef SD_CS_PIN
    { .id = Output_SdCardCS,     .port = GPIO_OUTPUT,      .pin = SD_CS_PIN,             .group = PinGroup_SdCard },
#endif
#ifdef SPI_CS_PIN
    { .id = Output_SPICS,        .port = GPIO_OUTPUT,      .pin = SPI_CS_PIN,            .group = PinGroup_SPI },
#endif
#if add_pin(SPI_RST)
    { .id = Output_SPIRST,       .port = SPI_RST_PORT,     .pin = SPI_RST_PIN,           .group = PinGroup_SPI },
#endif
#if add_aux_pin(SPINDLE, ENABLE)
    { .id = Output_SpindleOn,    .port = SPINDLE_ENABLE_PORT,    .pin = SPINDLE_ENABLE_PIN,    .group = PinGroup_SpindleControl},
#endif
#if add_aux_pin(SPINDLE, DIRECTION)
    { .id = Output_SpindleDir,   .port = SPINDLE_DIRECTION_PORT, .pin = SPINDLE_DIRECTION_PIN, .group = PinGroup_SpindleControl},
#endif
#if add_aux_pin(COOLANT, FLOOD)
    { .id = Output_CoolantFlood, .port = COOLANT_FLOOD_PORT,     .pin = COOLANT_FLOOD_PIN,     .group = PinGroup_Coolant},
#endif
#if add_aux_pin(COOLANT, MIST)
    { .id = Output_CoolantMist,  .port = COOLANT_MIST_PORT,      .pin = COOLANT_MIST_PIN,      .group = PinGroup_Coolant},
#endif
#ifdef LED_PIN
    { .id = Output_LED,          .port = GPIO_OUTPUT,      .pin = LED_PIN,               .group = PinGroup_LED },
#endif
#ifdef LED_R_PIN
    { .id = Output_LED_R,        .port = GPIO_OUTPUT,      .pin = LED_R_PIN,             .group = PinGroup_LED },
#endif
#ifdef LED_G_PIN
    { .id = Output_LED_G,        .port = GPIO_OUTPUT,      .pin = LED_G_PIN,             .group = PinGroup_LED },
#endif
#ifdef LED_B_PIN
    { .id = Output_LED_B,        .port = GPIO_OUTPUT,      .pin = LED_B_PIN,             .group = PinGroup_LED },
#endif
#ifdef LED_W_PIN
    { .id = Output_LED_W,        .port = GPIO_OUTPUT,      .pin = LED_W_PIN,             .group = PinGroup_LED },
#endif
#if add_pin (AUXOUTPUT0_PIN)
    { .id = Output_Aux0,         .port = AUXOUTPUT0_PORT,  .pin = AUXOUTPUT0_PIN,        .group = PinGroup_AuxOutput},
#endif
#if add_pin (AUXOUTPUT1_PIN)
    { .id = Output_Aux1,         .port = AUXOUTPUT1_PORT,  .pin = AUXOUTPUT1_PIN,        .group = PinGroup_AuxOutput},
#endif
#if add_pin (AUXOUTPUT2_PIN)
    { .id = Output_Aux2,         .port = AUXOUTPUT2_PORT,  .pin = AUXOUTPUT2_PIN,        .group = PinGroup_AuxOutput},
#endif
#if add_pin (AUXOUTPUT3_PIN)
    { .id = Output_Aux3,         .port = AUXOUTPUT3_PORT,  .pin = AUXOUTPUT3_PIN,        .group = PinGroup_AuxOutput},
#endif
#if add_pin (AUXOUTPUT4_PIN)
    { .id = Output_Aux4,         .port = AUXOUTPUT4_PORT,  .pin = AUXOUTPUT4_PIN,        .group = PinGroup_AuxOutput},
#endif
#if add_pin (AUXOUTPUT5_PIN)
    { .id = Output_Aux5,         .port = AUXOUTPUT5_PORT,  .pin = AUXOUTPUT5_PIN,        .group = PinGroup_AuxOutput},
#endif
#if add_pin (AUXOUTPUT6_PIN)
    { .id = Output_Aux6,         .port = AUXOUTPUT6_PORT,  .pin = AUXOUTPUT6_PIN,        .group = PinGroup_AuxOutput},
#endif
#if add_pin (AUXOUTPUT7_PIN)
    { .id = Output_Aux7,         .port = AUXOUTPUT7_PORT,  .pin = AUXOUTPUT7_PIN,        .group = PinGroup_AuxOutput},
#endif
#ifdef AUXOUTPUT0_PWM_PIN
    { .id = Output_Analog_Aux0, .port = GPIO_OUTPUT, .pin = AUXOUTPUT0_PWM_PIN, .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } },
#endif
#ifdef AUXOUTPUT1_PWM_PIN
    { .id = Output_Analog_Aux1, .port = GPIO_OUTPUT, .pin = AUXOUTPUT1_PWM_PIN, .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } },
#endif
#ifdef AUXOUTPUT2_PWM_PIN
    { .id = Output_Analog_Aux2, .port = GPIO_OUTPUT, .pin = AUXOUTPUT2_PWM_PIN, .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } },
#endif
#ifdef AUXOUTPUT3_PWM_PIN
    { .id = Output_Analog_Aux3, .port = GPIO_OUTPUT, .pin = AUXOUTPUT3_PWM_PIN, .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } },
#endif
};

#ifndef I2C_STROBE_BIT
#define I2C_STROBE_BIT 0
#endif

#ifndef SPI_IRQ_BIT
#define SPI_IRQ_BIT 0
#endif

#ifndef DEBOUNCE_DELAY
#define DEBOUNCE_DELAY 40 // ms
#endif

#if AUX_CONTROLS_ENABLED
static uint8_t probe_port;
static pin_debounce_t debounce = {0};
static void aux_irq_handler (uint8_t port, bool state);
#endif

#if SD_SHIFT_REGISTER
static step_dir_sr_t sd_sr;
#endif

#if OUT_SHIFT_REGISTER
static sr_reg_t sr16;
#endif

static struct {
    uint32_t length;
    uint32_t delay;
    uint32_t t_min_period; // timer ticks
    axes_signals_t out;
#if STEP_INJECT_ENABLE
    struct {
        hal_timer_t timer;
        axes_signals_t claimed;
        volatile axes_signals_t axes;
        volatile axes_signals_t out;
    } inject;
#endif
} step_pulse = {0};

static void systick_handler(void);
static void stepper_int_handler(void);
static void rpm_int_handler(void);
static void gpio_int_handler(uint gpio, uint32_t events);

#if I2C_STROBE_ENABLE || SPI_IRQ_BIT

#if I2C_STROBE_ENABLE
static driver_irq_handler_t i2c_strobe = { .type = IRQ_I2C_Strobe };
#endif

#if SPI_IRQ_BIT
static driver_irq_handler_t spi_irq = { .type = IRQ_SPI };
#endif

static bool irq_claim (irq_type_t irq, uint_fast8_t id, irq_callback_ptr handler)
{
    bool ok = false;

    switch(irq) {

#if I2C_STROBE_ENABLE
        case IRQ_I2C_Strobe:
            if((ok = i2c_strobe.callback == NULL))
                i2c_strobe.callback = handler;
            break;
#endif

#if SPI_IRQ_BIT
        case IRQ_SPI:
            if((ok = spi_irq.callback == NULL))
                spi_irq.callback = handler;
            break;
#endif

        default:
            break;
    }

    return ok;
}

#endif // I2C_STROBE_ENABLE || SPI_IRQ_BIT

static int64_t delay_callback (alarm_id_t id, void *callback)
{
    ((delay_callback_ptr)callback)();

    return 0;
}

static void driver_delay (uint32_t ms, delay_callback_ptr callback)
{
    if (ms > 0) {
        if (callback)
            add_alarm_in_ms(ms, delay_callback, callback, false);
        else {
            uint32_t delay = ms * 1000, start = timer_hw->timerawl;
            while (timer_hw->timerawl - start < delay)
                grbl.on_execute_delay(state_get());
        }
    }
    else if (callback)
        callback();
    tight_loop_contents();
}

//*************************  STEPPER  *************************//

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable, bool hold)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
#if TRINAMIC_ENABLE && TRINAMIC_I2C
    axes_signals_t tmc_enable = trinamic_stepper_enable(enable);

#elif defined(STEPPERS_ENABLE_PIN)

 #if STEPPERS_ENABLE_PORT == EXPANDER_PORT
   EXPANDER_OUT(STEPPERS_ENABLE_PIN, enable.x);
 #else
   DIGITAL_OUT(STEPPERS_ENABLE_PIN, enable.x);
 #endif

#else

 #ifdef XY_ENABLE_PIN
  #if XY_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(XY_ENABLE_PIN, enable.x || enable.y);
  #else
    DIGITAL_OUT(XY_ENABLE_PIN, enable.x || enable.y);
  #endif
 #else
  #ifdef X_ENABLE_PIN
   #if X_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(X_ENABLE_PIN, enable.x);
   #else
    DIGITAL_OUT(X_ENABLE_PIN, enable.x);
   #endif
  #endif
 #endif
 #ifdef Y_ENABLE_PIN
  #if Y_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(Y_ENABLE_PIN, enable.y);
  #else
    DIGITAL_OUT(Y_ENABLE_PIN, enable.y);
  #endif
 #endif

 #ifdef X2_ENABLE_PIN
  #if X2_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(X2_ENABLE_PIN, enable.x);
  #else
    DIGITAL_OUT(X2_ENABLE_PIN, enable.x);
  #endif
 #endif
 #ifdef Y2_ENABLE_PIN
  #if Y2_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(Y2_ENABLE_PIN, enable.y);
  #else
    DIGITAL_OUT(Y2_ENABLE_PIN, enable.y);
  #endif
 #endif

 #ifdef Z_ENABLE_PIN
  #if Z_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(Z_ENABLE_PIN, enable.z);
  #else
    DIGITAL_OUT(Z_ENABLE_PIN, enable.z);
  #endif
 #endif
 #ifdef Z2_ENABLE_PIN
  #if Z2_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(Z2_ENABLE_PIN, enable.z);
  #else
    DIGITAL_OUT(Z2_ENABLE_PIN, enable.z);
  #endif
 #endif

 #ifdef A_ENABLE_PIN
  #if A_ENABLE_PORT == EXPANDER_PORT
     EXPANDER_OUT(A_ENABLE_PIN, enable.a);
  #else
    DIGITAL_OUT(A_ENABLE_PIN, enable.a);
  #endif
 #endif

 #ifdef B_ENABLE_PIN
  #if B_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(B_ENABLE_PIN, enable.b);
  #else
   DIGITAL_OUT(B_ENABLE_PIN, enable.b);
  #endif
 #endif

 #ifdef C_ENABLE_PIN
  #if C_ENABLE_PORT == EXPANDER_PORT
    EXPANDER_OUT(C_ENABLE_PIN, enable.c);
  #else
    DIGITAL_OUT(C_ENABLE_PIN, enable.c);
  #endif
 #endif
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    hal.stepper.enable((axes_signals_t){AXES_BITMASK}, false);
    stepper_timer_set_period(pio1, stepper_timer_sm, stepper_timer_sm_offset, hal.f_step_timer / 50, 0); // ~2ms delay to allow drivers time to wake up.
    irq_set_enabled(PIO1_IRQ_0, true);
}

// Sets up stepper driver interrupt timeout, "Normal" version
static void __not_in_flash_func(stepperCyclesPerTick)(uint32_t cycles_per_tick)
{
    stepper_timer_set_period(pio1, stepper_timer_sm, stepper_timer_sm_offset, cycles_per_tick < 1000000 ? max(cycles_per_tick, step_pulse.t_min_period) : 1000000, PIO_RATE_ADJ);
}

#if STEP_PORT == GPIO_PIO || STEP_PORT == GPIO_PIO_1

// Set stepper pulse output pins
// NOTE: step_out are: bit0 -> X, bit1 -> Y, bit2 -> Z...
static inline __attribute__((always_inline)) void stepper_set_step (uint_fast8_t axis, pio_steps_t *pio_steps)
{
    switch(axis) {

        case X_AXIS:
#if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(x_step_pio, x_step_sm, pio_steps->value);
  #if X_GANGED
            step_pulse_generate(x2_step_pio, x2_step_sm, pio_steps->value);
  #endif
#else
            pio_steps->set |= (1 << (X_STEP_PIN - STEP_PINS_BASE));
  #if X_GANGED
            pio_steps->set |= (1 << (X2_STEP_PIN - STEP_PINS_BASE));
  #endif
#endif
            break;

        case Y_AXIS:
#if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(y_step_pio, y_step_sm, pio_steps->value);
  #if Y_GANGED
            step_pulse_generate(y2_step_pio, y2_step_sm, pio_steps->value);
  #endif
#else
            pio_steps->set |= (1 << (Y_STEP_PIN - STEP_PINS_BASE));
  #if Y_GANGED
            pio_steps->set |= (1 << (Y2_STEP_PIN - STEP_PINS_BASE));
  #endif
#endif
            break;

        case Z_AXIS:
#if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(z_step_pio, z_step_sm, pio_steps->value);
  #if Z_GANGED
            step_pulse_generate(z2_step_pio, z2_step_sm, pio_steps->value);
  #endif
#else
            pio_steps->set |= (1 << (Z_STEP_PIN - STEP_PINS_BASE));
  #if Z_GANGED
            pio_steps->set |= (1 << (Z2_STEP_PIN - STEP_PINS_BASE));
  #endif
#endif
            break;
#ifdef A_AXIS
        case A_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(a_step_pio, a_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (A_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
#ifdef B_AXIS
        case B_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(b_step_pio, b_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (B_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
#ifdef C_AXIS
        case C_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(c_step_pio, c_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (C_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
#ifdef U_AXIS
        case U_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(u_step_pio, u_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (U_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
#ifdef V_AXIS
        case V_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(v_step_pio, v_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (V_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
    }
}

#endif // STEP_PORT == GPIO_PIO || STEP_PORT == GPIO_PIO_1

#ifdef SQUARING_ENABLED

static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};

#if STEP_PORT == GPIO_PIO || STEP_PORT == GPIO_PIO_1

static inline __attribute__((always_inline)) void stepper_step_out1 (uint_fast8_t axis, pio_steps_t *pio_steps)
{
    switch(axis) {

        case X_AXIS:
#if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(x_step_pio, x_step_sm, pio_steps->value);
#else
            pio_steps->set |= (1 << (X_STEP_PIN - STEP_PINS_BASE));
#endif
            break;

        case Y_AXIS:
#if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(y_step_pio, y_step_sm, pio_steps->value);
#else
            pio_steps->set |= (1 << (Y_STEP_PIN - STEP_PINS_BASE));
#endif
            break;

        case Z_AXIS:
#if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(z_step_pio, z_step_sm, pio_steps->value);
#else
            pio_steps->set |= (1 << (Z_STEP_PIN - STEP_PINS_BASE));
#endif
            break;
#ifdef A_AXIS
        case A_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(a_step_pio, a_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (A_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
#ifdef B_AXIS
        case B_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(b_step_pio, b_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (B_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
#ifdef C_AXIS
        case C_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(c_step_pio, c_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (C_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
#ifdef U_AXIS
        case U_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(u_step_pio, u_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (U_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
#ifdef V_AXIS
        case V_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(v_step_pio, v_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (V_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
    }
}

static inline __attribute__((always_inline)) void stepper_step_out2 (uint_fast8_t axis, pio_steps_t *pio_steps)
{
    switch(axis) {

#if X_GANGED
        case X_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(x2_step_pio, x2_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (X2_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
#if Y_GANGED
        case Y_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(y2_step_pio, y2_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (Y2_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
#if Z_GANGED
        case Z_AXIS:
  #if STEP_PORT == GPIO_PIO_1
            step_pulse_generate(z2_step_pio, z2_step_sm, pio_steps->value);
  #else
            pio_steps->set |= (1 << (Z2_STEP_PIN - STEP_PINS_BASE));
  #endif
            break;
#endif
    }
}

#endif

// Set stepper pulse output pins
// NOTE: step_out are: bit0 -> X, bit1 -> Y, bit2 -> Z...
inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_out)
{
#if STEP_INJECT_ENABLE
    step_out.bits &= ~step_pulse.inject.claimed.bits;
#endif

    axes_signals_t step_out1 = { .bits = (step_out.bits & motors_1.bits) },
                   step_out2 = { .bits = ((step_out.bits & motors_2.bits) & 0b111) };

#if STEP_PORT == GPIO_SR8

    step_out1.bits ^= settings.steppers.step_invert.bits;
    step_out2.bits ^= settings.steppers.step_invert.bits;

    sd_sr.set.x_step = step_out1.x;
    sd_sr.set.y_step = step_out1.y;
    sd_sr.set.z_step = step_out1.z;
#ifdef A_AXIS
    sd_sr.set.m3_step = step_out1.a;
#elif X_GANGED
    sd_sr.set.m3_step = step_out2.x;
#elif Y_GANGED
    sd_sr.set.m3_step = step_out2.y;
#elif Z_GANGED
    sd_sr.set.m3_step = step_out2.z;
#endif

    step_dir_sr4_write(sr8_pio, sr8_sm, sd_sr.value);

#else // GPIO_PIO, GPIO_PIO_1

    uint_fast8_t idx = 0, mask = 1;

  #if STEP_PORT == GPIO_PIO
    pio_steps.set = 0;
  #elif STEP_PORT == GPIO_PIO_1
    pio_steps.set = 1;
  #endif

    while(step_out.bits) {

        if(step_out2.bits & mask)
            stepper_step_out2(idx, &pio_steps);

        if(step_out1.bits & mask)
            stepper_step_out1(idx, &pio_steps);

        idx++;
        mask <<= 1;
        step_out.bits >>= 1;
    }

  #if STEP_PORT == GPIO_PIO
    step_pulse_generate(step_pio, step_sm, pio_steps.value);
  #endif

#endif // GPIO_PIO, GPIO_PIO_1
}

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_1.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
    motors_2.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
}

#else // !SQUARING_ENABLED

inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_out)
{
#if STEP_INJECT_ENABLE
    step_out.bits &= ~step_pulse.inject.claimed.bits;
#endif

#if STEP_PORT == GPIO_SR8

    step_out.bits ^= settings.steppers.step_invert.bits;

    sd_sr.set.x_step = step_out.x;
    sd_sr.set.y_step = step_out.y;
    sd_sr.set.z_step = step_out.z;
  #ifdef A_AXIS
    sd_sr.set.m3_step = step_out.a;
  #elif X_GANGED
    sd_sr.set.m3_step = step_out.x;
  #elif Y_GANGED
    sd_sr.set.m3_step = step_out.y;
  #elif Z_GANGED
    sd_sr.set.m3_step = step_out.z;
  #endif

    step_dir_sr4_write(sr8_pio, sr8_sm, sd_sr.value);

#else // GPIO_PIO, GPIO_PIO_1

    uint_fast8_t idx = 0;

  #if STEP_PORT == GPIO_PIO
    pio_steps.set = 0;
 #elif STEP_PORT == GPIO_PIO_1
    pio_steps.set = 1;
  #endif

    while(step_out.bits) {
        if(step_out.bits & 0b1)
            stepper_set_step(idx, &pio_steps);
        idx++;
        step_out.bits >>= 1;
    }

  #if STEP_PORT == GPIO_PIO
    step_pulse_generate(step_pio, step_sm, pio_steps.value);
  #endif

#endif
}

#endif // SQUARING_ENABLED

#ifdef GANGING_ENABLED

static axes_signals_t getGangedAxes (bool auto_squared)
{
    axes_signals_t ganged = {0};

    if (auto_squared) {
#if X_AUTO_SQUARE
        ganged.x = On;
#endif
#if Y_AUTO_SQUARE
        ganged.y = On;
#endif
#if Z_AUTO_SQUARE
        ganged.z = On;
#endif
    } else {
#if X_GANGED
        ganged.x = On;
#endif

#if Y_GANGED
        ganged.y = On;
#endif

#if Z_GANGED
        ganged.z = On;
#endif
    }

    return ganged;
}

#endif

#if DIRECTION_PORT == GPIO_SR8

static inline __attribute__((always_inline)) void stepper_set_dir (uint_fast8_t axis, axes_signals_t dir_out, step_dir_sr_t *sd_sr)
{
    // dir signals are output on the next step pulse output

    switch(axis) {

        case X_AXIS:
            sd_sr->set.x_dir = sd_sr->reset.x_dir = dir_out.x;
#if X_GANGED
            sd_sr->set.m3_dir = sd_sr->reset.m3_dir = dir_out.x ^ settings.steppers.ganged_dir_invert.x;
#endif
            break;

        case Y_AXIS:
            sd_sr->set.y_dir = sd_sr->reset.y_dir = dir_out.y;
#if Y_GANGED
            sd_sr->set.m3_dir = sd_sr->reset.m3_dir = dir_out.y ^ settings.steppers.ganged_dir_invert.y;
#endif
            break;

        case Z_AXIS:
            sd_sr->set.x_dir = sd_sr->reset.z_dir = dir_out.z;
#if Z_GANGED
            sd_sr->set.m3_dir = sd_sr->reset.m3_dir = dir_out.z ^ settings.steppers.ganged_dir_invert.z;
#endif
            break;
#ifdef A_AXIS
        case A_AXIS:
            sd_sr->set.m3_dir = sd_sr->reset.m3_dir = dir_out.a;
            break;
#endif
    }
}

#else // DIRECTION_OUTMODE <= GPIO_MAP

static inline __attribute__((always_inline)) void stepper_set_dir (uint_fast8_t axis, axes_signals_t dir_out)
{
    switch(axis) {

        case X_AXIS:
            DIGITAL_OUT(X_DIRECTION_PIN, dir_out.x);
#if X_GANGED
            DIGITAL_OUT(X2_DIRECTION_PIN, dir_out.x);
#endif
            break;

        case Y_AXIS:
            DIGITAL_OUT(Y_DIRECTION_PIN, dir_out.y);
#if Y_GANGED
            DIGITAL_OUT(Y2_DIRECTION_PIN, dir_out.y);
#endif
            break;

        case Z_AXIS:
            DIGITAL_OUT(Z_DIRECTION_PIN, dir_out.x);
#if Z_GANGED
            DIGITAL_OUT(Z2_DIRECTION_PIN, dir_out.z);
#endif
            break;
#ifdef A_AXIS
        case A_AXIS:
            DIGITAL_OUT(A_DIRECTION_PIN, dir_out.a);
            break;
#endif
#ifdef B_AXIS
        case B_AXIS:
            DIGITAL_OUT(B_DIRECTION_PIN, dir_out.b);
            break;
#endif
#ifdef C_AXIS
        case C_AXIS:
            DIGITAL_OUT(C_DIRECTION_PIN, dir_out.c);
            break;
#endif
    }
}

#endif

// Set stepper direction output pins
// NOTE: see note for stepperSetStepOutputs()
//inline static __attribute__((always_inline)) void stepperSetDirOutputs (axes_signals_t dir_out)
static void stepperSetDirOutputs (axes_signals_t dir_out)
{
#if STEP_INJECT_ENABLE

    uint_fast8_t idx, mask = 1;
    axes_signals_t axes = { .bits = step_pulse.inject.claimed.bits };

    for(idx = 0; idx < N_AXIS; idx++) {
        if(!(axes.bits & mask))  {
  #if DIRECTION_PORT == GPIO_SR8
            stepper_set_dir(idx, dir_out, &sd_sr);
  #else
            stepper_set_dir(idx, dir_out);
  #endif
        }
        mask <<= 1;
    }

#elif DIRECTION_PORT == GPIO_SR8

    // dir signals are set on the next step pulse output

    dir_out.mask ^= settings.steppers.dir_invert.mask;
    sd_sr.set.x_dir = sd_sr.reset.x_dir = dir_out.x;
    sd_sr.set.y_dir = sd_sr.reset.y_dir = dir_out.y;
    sd_sr.set.z_dir = sd_sr.reset.z_dir = dir_out.z;
  #ifdef X2_DIRECTION_PIN
    sd_sr.set.m3_dir = sd_sr.reset.m3_dir = (dir_out.x ^ settings.steppers.ganged_dir_invert.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
    sd_sr.set.m3_dir = sd_sr.reset.m3_dir = (dir_out.y ^ settings.steppers.ganged_dir_invert.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    sd_sr.set.m3_dir = sd_sr.reset.m3_dir = (dir_out.z ^ settings.steppers.ganged_dir_invert.z);
  #endif
  #ifdef A_DIRECTION_PIN
    sd_sr.set.m3_dir = sd_sr.reset.m3_dir = dir_out.a;
  #endif

#elif DIRECTION_PORT == GPIO_OUTPUT

 #if DIRECTION_OUTMODE == GPIO_MAP
 
    gpio_put_masked(DIRECTION_MASK, dir_outmap[dir_out.mask]);

  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PIN, dir_out.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_PIN, dir_out.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_PIN, dir_out.z);
  #endif

 #else // GPIO_SHIFT<N>

    gpio_put_masked(DIRECTION_MASK, dir_out.mask << DIRECTION_OUTMODE);

  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PIN, dir_out.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_PIN, dir_out.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_PIN, dir_out.z);
  #endif
 #endif
 
#endif
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    irq_set_enabled(PIO1_IRQ_0, false);
    stepper_timer_stop(pio1, stepper_timer_sm);
    
    if(clear_signals) {
        stepperSetDirOutputs((axes_signals_t){0});
        stepperSetStepOutputs((axes_signals_t){0});
    }
}

// Sets stepper direction and pulse pins and starts a step pulse.
static void __not_in_flash_func(stepperPulseStart)(stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        stepperSetDirOutputs(stepper->dir_out);
    }

    if(stepper->step_out.bits)
        stepperSetStepOutputs(stepper->step_out);
}

#if STEP_INJECT_ENABLE

static void stepperClaimMotor (uint_fast8_t axis_id, bool claim)
{
    if(claim)
        step_pulse.inject.claimed.bits |= ((1 << axis_id) & AXES_BITMASK);
    else {
        step_pulse.inject.claimed.bits &= ~(1 << axis_id);
        step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits;
    }
}

void stepperOutputStep (axes_signals_t step_out, axes_signals_t dir_out)
{
    if(step_out.bits) {

        uint_fast8_t idx = N_AXIS - 1;
        axes_signals_t axes = { .bits = (step_out.bits & AXES_BITMASK) };
#if STEP_PORT == GPIO_SR8
        step_dir_sr_t sd_sr;
#else
        pio_steps_t steps = pio_steps;
        steps.set = 0;
#endif
        do {
            if(axes.bits & (1 << (N_AXIS - 1))) {
#if DIRECTION_PORT == GPIO_SR8
                stepper_set_dir(idx, dir_out, &sd_sr);
#else
                stepper_set_dir(idx, dir_out);
                stepper_set_step(idx, &steps);
#endif
            }
            idx--;
            axes.bits <<= 1;
            axes.bits &= AXES_BITMASK;
        } while(axes.bits);

#if STEP_PORT == GPIO_PIO

        step_pulse_generate(step_pio, step_sm, steps.value);

#elif STEP_PORT == GPIO_SR8

        step_out.bits ^= settings.steppers.step_invert.bits;

        sd_sr.set.x_step = step_out.x;
        sd_sr.set.y_step = step_out.y;
        sd_sr.set.z_step = step_out.z;
  #ifdef A_AXIS
        sd_sr.set.m3_step = step_out.a;
  #elif X_GANGED
        sd_sr.set.m3_step = step_out.x;
  #elif Y_GANGED
        sd_sr.set.m3_step = step_out.y;
  #elif Z_GANGED
        sd_sr.set.m3_step = step_out.z;
  #endif

    step_dir_sr4_write(sr8_pio, sr8_sm, sd_sr.value);

#endif
    }
}

#endif // STEP_INJECT_ENABLE

//*************************  LIMIT  *************************//

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    bool disable = !on;
    uint32_t i = limit_inputs.n_pins;
    axes_signals_t pin;
    limit_signals_t homing_source = xbar_get_homing_source_from_cycle(homing_cycle);

    on = on && settings.limits.flags.hard_enabled;

    do {
        i--;
        if(on && homing_cycle.mask) {
            pin = xbar_fn_to_axismask(limit_inputs.pins.inputs[i].id);
            disable = limit_inputs.pins.inputs[i].group == PinGroup_Limit ? (pin.mask & homing_source.min.mask) : (pin.mask & homing_source.max.mask);
        }
        pinEnableIRQ(&limit_inputs.pins.inputs[i], disable ? IRQ_Mode_None : limit_inputs.pins.inputs[i].mode.irq_mode);
    } while (i);

#if TRINAMIC_ENABLE
//    trinamic_homing(homing_cycle.mask != 0);
#endif
}

// Returns limit state as an limit_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState (void)
{
    limit_signals_t signals = {0};

    signals.min.x = DIGITAL_IN(X_LIMIT_PIN);
#ifdef X2_LIMIT_PIN
    signals.min2.x = DIGITAL_IN(X2_LIMIT_PIN);
#endif
#ifdef X_LIMIT_PIN_MAX
    signals.max.x = DIGITAL_IN(X_LIMIT_PIN_MAX);
#endif
    signals.min.y = DIGITAL_IN(Y_LIMIT_PIN);
#ifdef Y2_LIMIT_PIN
    signals.min2.y = DIGITAL_IN(Y2_LIMIT_PIN);
#endif
#ifdef Y_LIMIT_PIN_MAX
    signals.max.y = DIGITAL_IN(Y_LIMIT_PIN_MAX);
#endif
    signals.min.z = DIGITAL_IN(Z_LIMIT_PIN);
#ifdef Z2_LIMIT_PIN
    signals.min2.z = DIGITAL_IN(Z2_LIMIT_PIN);
#endif
#ifdef Z_LIMIT_PIN_MAX
    signals.max.z = DIGITAL_IN(Z_LIMIT_PIN_MAX);
#endif
#ifdef A_LIMIT_PIN
    signals.min.a = DIGITAL_IN(A_LIMIT_PIN);
#endif
#ifdef A_LIMIT_PIN_MAX
    signals.max.a = DIGITAL_IN(A_LIMIT_PIN_MAX);
#endif
#ifdef B_LIMIT_PIN
    signals.min.b = DIGITAL_IN(B_LIMIT_PIN);
#endif
#ifdef B_LIMIT_PIN_MAX
    signals.max.b = DIGITAL_IN(B_LIMIT_PIN_MAX);
#endif
#ifdef C_LIMIT_PIN
    signals.min.c = DIGITAL_IN(C_LIMIT_PIN);
#endif
#ifdef C_LIMIT_PIN_MAX
    signals.max.c = DIGITAL_IN(C_LIMIT_PIN_MAX);
#endif
#ifdef U_LIMIT_PIN
    signals.min.u = DIGITAL_IN(U_LIMIT_PIN);
#endif
#ifdef U_LIMIT_PIN_MAX
    signals.max.u = DIGITAL_IN(U_LIMIT_PIN_MAX);
#endif
#ifdef V_LIMIT_PIN
    signals.min.v = DIGITAL_IN(U_LIMIT_PIN);
#endif
#ifdef V_LIMIT_PIN_MAX
    signals.max.v = DIGITAL_IN(U_LIMIT_PIN_MAX);
#endif

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t __not_in_flash_func(systemGetState)(void)
{
    control_signals_t signals = {0};

#ifdef RESET_PIN
#ifdef ESTOP_ENABLE
    signals.e_stop = DIGITAL_IN(RESET_PIN);
#else
    signals.reset = DIGITAL_IN(RESET_PIN);
#endif
#endif
#ifdef FEED_HOLD_PIN
    signals.feed_hold = DIGITAL_IN(FEED_HOLD_PIN);
#endif
#ifdef CYCLE_START_PIN
    signals.cycle_start = DIGITAL_IN(CYCLE_START_PIN);
#endif
#if SAFETY_DOOR_BIT
    signals.safety_door_ajar = safety_door->active;
#endif

#if AUX_CONTROLS_ENABLED

  #ifdef SAFETY_DOOR_PIN
    if(debounce.safety_door)
        signals.safety_door_ajar = Off;
    else
        signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PIN);
  #endif
  #ifdef MOTOR_FAULT_PIN
    signals.motor_fault = DIGITAL_IN(MOTOR_FAULT_PIN);
  #endif
  #ifdef MOTOR_WARNING_PIN
    signals.motor_warning = DIGITAL_IN(MOTOR_WARNING_PIN);
  #endif

  #if AUX_CONTROLS_SCAN
    signals = aux_ctrl_scan_status(signals);
  #endif

#endif // AUX_CONTROLS_ENABLED

    return signals;
}

//*************************  PROBE  *************************//

#ifdef PROBE_PIN

// Toggle probe connected status. Used when no input pin is available.
static void probeConnectedToggle (void)
{
    probe.connected = !probe.connected;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;

    gpio_set_inover(PROBE_PIN, probe.inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

    if(hal.driver_cap.probe_latch) {
        probe.is_probing = Off;
        probe.triggered = hal.probe.get_state().triggered;
        pin_irq_mode_t irq_mode = probing && !probe.triggered ? (probe.inverted ? IRQ_Mode_Falling : IRQ_Mode_Rising) : IRQ_Mode_None;
        probe.irq_enabled = ioport_enable_irq(probe_port, irq_mode, aux_irq_handler) && irq_mode != IRQ_Mode_None;
    } else { 
        if((probe.irq_enabled = probing))
            gpio_set_irq_enabled(PROBE_PIN, probe.inverted ? GPIO_IRQ_LEVEL_LOW : GPIO_IRQ_LEVEL_HIGH, true);
        else
            gpio_set_irq_enabled(PROBE_PIN, GPIO_IRQ_ALL, false);
    }

    if(!probe.irq_enabled)
        probe.triggered = Off;

    probe.is_probing = probing;
}

// Returns the probe connected and triggered pin states.
static probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    state.triggered = probe.is_probing && probe.irq_enabled ? probe.triggered : DIGITAL_IN(PROBE_PIN);

    return state;
}

#endif // PROBE_PIN

#if MPG_ENABLE == 1

static input_signal_t *mpg_pin = NULL;

static void mpg_select (void *data)
{
    stream_mpg_enable(DIGITAL_IN(mpg_pin->pin) == 0);

    pinEnableIRQ(mpg_pin, (mpg_pin->mode.irq_mode = sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling));
}

static void mpg_enable (void *data)
{
    if (sys.mpg_mode != (DIGITAL_IN(mpg_pin->pin) == 0))
        mpg_select(data);
    else
        pinEnableIRQ(mpg_pin, (mpg_pin->mode.irq_mode = sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling));
}

#endif // MPG_ENABLE == 1

#if AUX_CONTROLS_ENABLED

static int64_t __not_in_flash_func(aux_irq_latch)(alarm_id_t id, void *input)
{
    control_signals_t signals = {0};

    switch (((aux_ctrl_t *)input)->function) {
        case Input_SafetyDoor:
            debounce.safety_door = Off;
            signals.safety_door_ajar = systemGetState().safety_door_ajar;
            break;
    }

    if(signals.mask)
        hal.control.interrupt_callback(signals);

    return 0;
}

static void aux_irq_handler (uint8_t port, bool state)
{
    aux_ctrl_t *pin;
    control_signals_t signals = {0};

    if((pin = aux_ctrl_get_pin(port))) {
        switch(pin->function) {
#ifdef PROBE_PIN
            case Input_Probe:
                if(probe.is_probing) {
                    probe.triggered = On;
                    return;
                } else
                    signals.probe_triggered = On;
                break;
#endif
#ifdef I2C_STROBE_PIN
            case Input_I2CStrobe:
                if(i2c_strobe.callback)
                    i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PIN) == 0);
                break;
#endif
#ifdef MPG_MODE_PIN
            case Input_MPGSelect:
                task_add_immediate(mpg_select, NULL);
                break;
#endif
            default:
                break;
        }
        signals.mask |= pin->cap.mask;
        if(pin->irq_mode == IRQ_Mode_Change && pin->function != Input_Probe)
            signals.deasserted = hal.port.wait_on_input(Port_Digital, pin->aux_port, WaitMode_Immediate, 0.0f) == 0;
    }

    if(signals.mask) {
        if(!signals.deasserted)
            signals.mask |= systemGetState().mask;
        hal.control.interrupt_callback(signals);
    }
}

static bool aux_claim_explicit (aux_ctrl_t *aux_ctrl)
{
    xbar_t *pin;

    if((pin = ioport_claim(Port_Digital, Port_Input, &aux_ctrl->aux_port, NULL))) {
        ioport_set_function(pin, aux_ctrl->function, &aux_ctrl->cap);
#ifdef PROBE_PIN
        if(aux_ctrl->function == Input_Probe) {
            probe_port = aux_ctrl->aux_port;
            hal.probe.get_state = probeGetState;
            hal.probe.configure = probeConfigure;
            hal.probe.connected_toggle = probeConnectedToggle;
            hal.driver_cap.probe_pull_up = On;
            hal.signals_cap.probe_triggered = hal.driver_cap.probe_latch = (pin->cap.irq_mode & aux_ctrl->irq_mode) == aux_ctrl->irq_mode;
        }
#endif
#ifdef MPG_MODE_PIN
        if(aux_ctrl->function == Input_MPGSelect)
            mpg_pin = (input_signal_t *)aux_ctrl->input;
#endif
#if defined(SAFETY_DOOR_PIN)
        if(aux_ctrl->function == Input_SafetyDoor)
            ((input_signal_t *)aux_ctrl->input)->mode.debounce = hal.driver_cap.software_debounce;
#endif
    } else
        aux_ctrl->aux_port = 0xFF;

    return aux_ctrl->aux_port != 0xFF;
}

#endif // AUX_CONTROLS_ENABLED

#if AUX_CONTROLS

bool aux_out_claim_explicit (aux_ctrl_out_t *aux_ctrl)
{
    xbar_t *pin;

#ifdef USE_EXPANDERS
    if(aux_ctrl->port == (void *)EXPANDER_PORT) {
        if((iox_out[aux_ctrl->pin] = malloc(sizeof(xbar_t))))
            memcpy(iox_out[aux_ctrl->pin], aux_ctrl->output, sizeof(xbar_t));
        else
            aux_ctrl->aux_port = 0xFF;
    } else
#endif
    if((pin = ioport_claim(Port_Digital, Port_Output, &aux_ctrl->aux_port, NULL))) {
        if(aux_ctrl->function == Output_SpindlePWM || aux_ctrl->function == Output_Spindle1PWM) {
            gpio_init(aux_ctrl->pin);
            gpio_set_dir_out_masked(1 << aux_ctrl->pin);
            gpio_set_function(aux_ctrl->pin, GPIO_FUNC_PWM);
        }
        ioport_set_function(pin, aux_ctrl->function, NULL);
    } else
        aux_ctrl->aux_port = 0xFF;

    return aux_ctrl->aux_port != 0xFF;
}

#endif // AUX_CONTROLS

//*************************  SPINDLE  *************************//

#if DRIVER_SPINDLE_ENABLE

// Static spindle (off, on cw & on ccw)

#if SPINDLE_PORT == GPIO_OUTPUT

inline static void spindle_off (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    spindle->context.pwm->flags.enable_out = Off;
  #ifdef SPINDLE_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned)
        DIGITAL_OUT(SPINDLE_DIRECTION_PIN, Off);
    else
        DIGITAL_OUT(SPINDLE_ENABLE_PIN, Off);
  #elif defined(SPINDLE_ENABLE_PIN)
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, Off);
  #endif
#elif defined(SPINDLE_ENABLE_PIN)
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, Off);
#endif
}

inline static void spindle_on (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
  #ifdef SPINDLE_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned)
        DIGITAL_OUT(SPINDLE_DIRECTION_PIN, On);
    else
        DIGITAL_OUT(SPINDLE_ENABLE_PIN, On);
  #elif defined(SPINDLE_ENABLE_PIN)
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, On);
  #endif
  #if SPINDLE_ENCODER_ENABLE
    if(!spindle->context.pwm->flags.enable_out && spindle->reset_data)
        spindle->reset_data();
  #endif
    spindle->context.pwm->flags.enable_out = On;
#elif defined(SPINDLE_ENABLE_PIN)
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, On);
#endif
}

inline static void spindle_dir (bool ccw)
{
#ifdef SPINDLE_DIRECTION_PIN
    DIGITAL_OUT(SPINDLE_DIRECTION_PIN, ccw);
#else
    UNUSED(ccw);
#endif
}

#elif SPINDLE_PORT == EXPANDER_PORT

inline static void spindle_off (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    spindle->context.pwm->flags.enable_out = Off;
    if(spindle->context.pwm->flags.cloned) {
        EXPANDER_OUT(SPINDLE_DIRECTION_PIN, settings.pwm_spindle.invert.ccw);
    } else {
        EXPANDER_OUT(SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
    }
#else
    EXPANDER_OUT(SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
#endif
}

inline static void spindle_on (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    if(spindle->context.pwm->flags.cloned) {
        EXPANDER_OUT(SPINDLE_DIRECTION_PIN, !settings.pwm_spindle.invert.ccw);
    } else {
        EXPANDER_OUT(SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
    }
  #if SPINDLE_ENCODER_ENABLE
    if(!spindle->context.pwm->flags.enable_out && spindle->reset_data)
        spindle->reset_data();
  #endif
    spindle->context.pwm->flags.enable_out = On;
#else
    EXPANDER_OUT(SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
#endif
}

inline static void spindle_dir (bool ccw)
{
    EXPANDER_OUT(SPINDLE_DIRECTION_PIN, ccw ^ settings.pwm_spindle.invert.ccw);
 //   UNUSED(ccw);
}

#endif

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!state.on)
        spindle_off(spindle);
    else {
        spindle_dir(state.ccw);
        spindle_on(spindle);
    }
}

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

// Variable spindle control functions

static void pwm_off (spindle_ptrs_t *spindle)
{
    if (spindle->context.pwm->flags.always_on)
        pwm_set_gpio_level(SPINDLE_PWM_PIN, spindle->context.pwm->off_value);
    else
        pwm_set_gpio_level(SPINDLE_PWM_PIN, spindle->context.pwm->off_value);
}

// Sets spindle speed
static void __not_in_flash_func(spindleSetSpeed)(spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if(pwm_value == spindle->context.pwm->off_value) {

        if(spindle->context.pwm->flags.rpm_controlled) {
            spindle_off(spindle);
            if(spindle->context.pwm->flags.laser_off_overdrive)
                pwm_set_gpio_level(SPINDLE_PWM_PIN, spindle->context.pwm->pwm_overdrive);
        } else
            pwm_off(spindle);

    } else {

        if(!spindle->context.pwm->flags.enable_out && spindle->context.pwm->flags.rpm_controlled)
            spindle_on(spindle);

        pwm_set_gpio_level(SPINDLE_PWM_PIN, pwm_value);
    }
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!(spindle->context.pwm->flags.cloned ? state.ccw : state.on)) {
        spindle_off(spindle);
        pwm_off(spindle);
    } else {
#if defined(SPINDLE_DIRECTION_PIN)
        if(!spindle->context.pwm->flags.cloned)
            spindle_dir(state.ccw);
#endif
        if(rpm == 0.0f && spindle->context.pwm->flags.rpm_controlled)
            spindle_off(spindle);
        else {
            spindle_on(spindle);
            spindleSetSpeed(spindle, spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false));
        }
    }
}

bool spindleConfig (spindle_ptrs_t *spindle)
{
    if (spindle == NULL)
        return false;

    uint32_t prescaler = settings.pwm_spindle.pwm_freq > 2000.0f ? 1 : (settings.pwm_spindle.pwm_freq > 200.0f ? 12 : 50);

    if(spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, clock_get_hz(clk_sys) / prescaler)) {

        spindle->set_state = spindleSetStateVariable;

        // Get the default config for
        pwm_config config = pwm_get_default_config();

        // Set divider, not using the 4 fractional bit part of the clock divider, only the integer part
        pwm_config_set_clkdiv_int(&config, prescaler);
        // Set the top value of the PWM => the period
        pwm_config_set_wrap(&config, spindle_pwm.period);
        // Set the off value of the PWM => off duty cycle (either 0 or the off value)
        pwm_set_gpio_level(SPINDLE_PWM_PIN, spindle_pwm.off_value);

        // Set polarity of the channel
    //    uint channel = pwm_gpio_to_channel(SPINDLE_PWM_PIN);                                                                                // Get which is associated with the PWM pin
    //    pwm_config_set_output_polarity(&config, (!channel & settings.pwm_spindle.invert.pwm), (channel & settings.pwm_spindle.invert.pwm)); // Set the polarity of the pin's channel

        // Load the configuration into our PWM slice, and set it running.
        pwm_init(pwm_gpio_to_slice_num(SPINDLE_PWM_PIN), &config, true);
    } else {
        if(spindle->context.pwm->flags.enable_out)
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        spindle->set_state = spindleSetState;
    }

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

#if PPI_ENABLE

static spindle_ptrs_t *ppi_spindle;

static void spindlePulseOn (spindle_ptrs_t *spindle, uint_fast16_t pulse_length)
{
    //    PPI_TIMER->ARR = pulse_length;
    //    PPI_TIMER->EGR = TIM_EGR_UG;
    //    PPI_TIMER->CR1 |= TIM_CR1_CEN;
    spindle_on((ppi_spindle = spindle));
}

#endif // PPI_ENABLE

#endif // DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = { settings.pwm_spindle.invert.mask };

    UNUSED(spindle);

#if SPINDLE_PORT == GPIO_OUTPUT

#ifdef SPINDLE_ENABLE_PIN
    state.on = DIGITAL_IN(SPINDLE_ENABLE_PIN);
#else
    state.on = pwmEnabled ^ settings.pwm_spindle.invert.on;
#endif
#ifdef SPINDLE_DIRECTION_PIN
    state.ccw = DIGITAL_IN(SPINDLE_DIRECTION_PIN);
#endif

#elif SPINDLE_PORT == EXPANDER_PORT

    state.on = EXPANDER_IN(SPINDLE_ENABLE_PIN);
    state.ccw = EXPANDER_IN(SPINDLE_DIRECTION_PIN);

#endif

    state.value ^= settings.pwm_spindle.invert.mask;

#ifdef SPINDLE_PWM_PIN
    state.on |= spindle->param->state.on;
#endif

#if SPINDLE_ENCODER_ENABLE
    if(spindle->get_data) {
        spindle_data_t *spindle_data = spindle->get_data(SpindleData_AtSpeed);
        state.at_speed = spindle_data->state_programmed.at_speed;
        state.encoder_error = spindle_data->state_programmed.encoder_error;
    }
#endif

    return state;
}

#endif // DRIVER_SPINDLE_ENABLE

#if SPINDLE_ENCODER_ENABLE

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    static uint32_t pl[4] = {};
    static uint_fast8_t idx = 0;
    static volatile bool running = false;

 //   uint32_t rpm_timer_delta;
    spindle_encoder_counter_t encoder;

//    while(spindle_encoder.spin_lock);

    __disable_irq();
    memcpy(&encoder, &spindle_encoder.counter, sizeof(spindle_encoder_counter_t));
    __enable_irq();

    uint32_t tval = (uint32_t)(get_absolute_time() - encoder_started);
    uint16_t cval = pwm_get_counter(encoder_pwm);

    spindle_encoder.counter.pulse_count = (encoder_ovf | cval);

    if(tval - spindle_encoder.timer.last_pulse > 100000) {

        spindle_encoder.timer.pulse_length = (tval - spindle_encoder.timer.last_pulse) /
                                              (spindle_encoder.counter.pulse_count -
                                                spindle_encoder.counter.last_count) ;

        if(spindle_encoder.timer.pulse_length == 0) {
            running = false;
            spindle_data.rpm = 0.0f;
    //        rpm_timer_delta = (uint16_t)((pwm_get_counter(encoder_pwm) - (uint16_t)encoder.last_count)) * pulse_length;
        } else if(!running) {
            running = true;
            pl[0] = pl[1] = pl[2] = pl[3] = spindle_encoder.timer.pulse_length;
        } else {
            pl[idx++] = spindle_encoder.timer.pulse_length;
            idx &= 0b11;
        }

        spindle_encoder.counter.last_count = spindle_encoder.counter.pulse_count;
        spindle_encoder.timer.last_pulse = tval;
    }
 
    switch(request) {

        case SpindleData_Counters:
            spindle_data.index_count = encoder.index_count;
            spindle_data.pulse_count = spindle_encoder.counter.pulse_count;
            spindle_data.error_count = spindle_encoder.error_count;
            break;

        case SpindleData_RPM:
            if(running)
                spindle_data.rpm = spindle_encoder.rpm_factor / (float)((pl[0] + pl[1] + pl[2] + pl[3]) >> 2);
            break;

        case SpindleData_AtSpeed:
            if(running)
                spindle_data.rpm = spindle_encoder.rpm_factor / (float)((pl[0] + pl[1] + pl[2] + pl[3]) >> 2);
            spindle_data.state_programmed.at_speed = !spindle_data.at_speed_enabled || (spindle_data.rpm >= spindle_data.rpm_low_limit && spindle_data.rpm <= spindle_data.rpm_high_limit);
            spindle_data.state_programmed.encoder_error = spindle_encoder.error_count > 0;
            break;

        case SpindleData_AngularPosition:
            spindle_data.angular_position = (float)encoder.index_count +
                                             (float)(spindle_encoder.counter.pulse_count - encoder.last_index) *
                                                      spindle_encoder.pulse_distance;
            break;
    }

    return &spindle_data;
}

static void spindleDataReset (void)
{
    while(spindle_encoder.spin_lock);

    uint32_t timeout = elapsed_ticks + 1000; // 1 second

    uint32_t index_count = spindle_encoder.counter.index_count + 2;
    if(spindleGetData(SpindleData_RPM)->rpm > 0.0f) { // wait for index pulse if running

        while(index_count != spindle_encoder.counter.index_count && elapsed_ticks <= timeout);

//        if(elapsed_ticks > timeout)
//            alarm?
    }

    encoder_ovf = 0;
    encoder_started = get_absolute_time();
    pwm_set_irq_enabled(encoder_pwm, false);
    pwm_clear_irq(encoder_pwm);
    pwm_set_counter(encoder_pwm, 0);
    pwm_set_irq_enabled(encoder_pwm, true);

    spindle_encoder.timer.last_index =
    spindle_encoder.timer.pulse_length =
    spindle_encoder.counter.last_count =
    spindle_encoder.counter.last_index =
    spindle_encoder.counter.pulse_count =
    spindle_encoder.counter.index_count =
    spindle_encoder.error_count = 0;
}

static void onSpindleProgrammed (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode)
{
    if(on_spindle_programmed)
        on_spindle_programmed(spindle, state, rpm, mode);

    if(spindle->get_data == spindleGetData) {
        spindle_set_at_speed_range(spindle, &spindle_data, rpm);
        spindle_data.state_programmed.on = state.on;
        spindle_data.state_programmed.ccw = state.ccw;
    }
}

#endif // SPINDLE_ENCODER_ENABLE

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
#if COOLANT_PORT == GPIO_OUTPUT

#ifdef COOLANT_FLOOD_PIN
    DIGITAL_OUT(COOLANT_FLOOD_PIN, mode.flood);
#endif
#ifdef COOLANT_MIST_PIN
    DIGITAL_OUT(COOLANT_MIST_PIN, mode.mist);
#endif

#elif COOLANT_PORT == EXPANDER_PORT

    mode.value ^= settings.coolant.invert.mask;
    EXPANDER_OUT(COOLANT_FLOOD_PIN, mode.flood);
#ifdef COOLANT_MIST_PIN
    EXPANDER_OUT(COOLANT_MIST_PIN, mode.mist);
#endif

#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = { settings.coolant.invert.mask };

#if COOLANT_PORT == GPIO_OUTPUT

#ifdef COOLANT_FLOOD_PIN
    state.flood = DIGITAL_IN(COOLANT_FLOOD_PIN);
#endif
#ifdef COOLANT_MIST_PIN
    state.mist = DIGITAL_IN(COOLANT_MIST_PIN);
#endif

#elif COOLANT_PORT == EXPANDER_PORT
    state.flood = EXPANDER_IN(COOLANT_FLOOD_PIN);
#ifdef COOLANT_MIST_PIN
    state.mist = EXPANDER_IN(COOLANT_MIST_PIN);
#endif

#endif

    state.value ^= settings.coolant.invert.mask;

    return state;
}

#if SPI_RST_PORT == EXPANDER_PORT

void spi_reset_out (bool on)
{
    EXPANDER_OUT(SPI_RST_PIN, on);
}

#endif

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    *ptr |= bits;
    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    __enable_irq();

    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    __enable_irq();

    return prev;
}

static uint64_t getElapsedMicros (void)
{
    return to_us_since_boot(get_absolute_time());
}

static uint32_t getElapsedTicks (void)
{
    return elapsed_ticks;
}

void pinEnableIRQ (const input_signal_t *input, pin_irq_mode_t irq_mode)
{
    switch (irq_mode) {

        case IRQ_Mode_Rising:
            gpio_set_irq_enabled(input->pin, GPIO_IRQ_EDGE_RISE, true);
            break;

        case IRQ_Mode_Falling:
            gpio_set_irq_enabled(input->pin, GPIO_IRQ_EDGE_FALL, true);
            break;

        case IRQ_Mode_Change:
            gpio_set_irq_enabled(input->pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
            break;

        case IRQ_Mode_Low:
            gpio_set_irq_enabled(input->pin, GPIO_IRQ_LEVEL_LOW, true);
            break;

        case IRQ_Mode_High:
            gpio_set_irq_enabled(input->pin, GPIO_IRQ_LEVEL_HIGH, true);
            break;

        case IRQ_Mode_None:
            gpio_set_irq_enabled(input->pin, GPIO_IRQ_ALL, false);
            break;
    }
}

// Configures peripherals when settings are initialized or changed
void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
#if USE_STEPDIR_MAP

    // Signal inversion is done in hardware!
    axes_signals_t step_invert = { .value = settings->steppers.step_invert.value },
                   dir_invert = { .value = settings->steppers.dir_invert.value };

    settings->steppers.step_invert.value = settings->steppers.dir_invert.value = 0;

    stepdirmap_init(settings);

    settings->steppers.step_invert = step_invert;
    settings->steppers.dir_invert = dir_invert;

#endif

    if (IOInitDone) {

#if WIFI_ENABLE
        if(!net.wifi)
            net.wifi = wifi_start();
#endif

#if ETHERNET_ENABLE

        static bool enet_init = true;

        if(enet_init && !net.ethernet) {
            enet_init = false;
            net.ethernet = enet_start();
        }
#endif

#if BLUETOOTH_ENABLE == 1
        if(!net.bluetooth)
            net.bluetooth = bluetooth_start_local();
#endif

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
        if(changed.spindle) {
            spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
            if(spindle_id == spindle_get_default())
                spindle_select(spindle_id);
        }
#endif

#if SPINDLE_ENCODER_ENABLE

        static const spindle_data_ptrs_t encoder_data = {
            .get = spindleGetData,
            .reset = spindleDataReset
        };

        static bool event_claimed = false;

        if((hal.spindle_data.get = settings->spindle.ppr > 0 ? spindleGetData : NULL)) {
            if(spindle_encoder.ppr != settings->spindle.ppr) {

                spindle_ptrs_t *spindle;

                hal.spindle_data.reset = spindleDataReset;
                if((spindle = spindle_get(0)))
                    spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);

                if(!event_claimed) {
                    event_claimed = true;
                    on_spindle_programmed = grbl.on_spindle_programmed;
                    grbl.on_spindle_programmed = onSpindleProgrammed;
                }

                spindle_encoder.ppr = settings->spindle.ppr;
                spindle_encoder.tics_per_irq = max(1, spindle_encoder.ppr / 32);
                spindle_encoder.pulse_distance = 1.0f / spindle_encoder.ppr;
                spindle_encoder.maximum_tt = 250000UL / RPM_TIMER_RESOLUTION; // 250ms
                spindle_encoder.rpm_factor = (60.0f * 1000000.0f / RPM_TIMER_RESOLUTION) / (float)spindle_encoder.ppr;
                spindleDataReset();
            }
        } else {
            spindle_encoder.ppr = 0;
            hal.spindle_data.reset = NULL;
        }

        spindle_bind_encoder(spindle_encoder.ppr ? &encoder_data : NULL);

#endif // SPINDLE_ENCODER_ENABLE

#ifdef NEOPIXELS_PIN

        if(neopixel.leds == NULL || hal.rgb0.num_devices != settings->rgb_strip.length0) {

            if(settings->rgb_strip.length0 == 0)
                settings->rgb_strip.length0 = hal.rgb0.num_devices;
            else
                hal.rgb0.num_devices = settings->rgb_strip.length0;

            if(neopixel.leds) {
                free(neopixel.leds);
                neopixel.leds = NULL;
            }

            if(hal.rgb0.num_devices) {
                neopixel.num_bytes = hal.rgb0.num_devices * sizeof(uint32_t);
                if((neopixel.leds = calloc(neopixel.num_bytes, sizeof(uint8_t))) == NULL)
                    hal.rgb0.num_devices = 0;
            }

            neopixel.num_leds = hal.rgb0.num_devices;
        }

#endif // NEOPIXELS_PIN

        step_pulse.t_min_period = (uint32_t)ceilf((settings->steppers.pulse_microseconds + STEP_PULSE_TOFF_MIN) * (hal.f_step_timer / 1000000.0f));

#if SD_SHIFT_REGISTER
        pio_steps.length = (uint32_t)(10.0f * (settings->steppers.pulse_microseconds - 0.8f));
        pio_steps.delay = settings->steppers.pulse_delay_microseconds <= 0.8f
                           ? 2
                           : (uint32_t)(10.0f * (settings->steppers.pulse_delay_microseconds - 0.8f));
        sr_delay_set(sr8_delay_pio, sr8_delay_sm, pio_steps.delay);
        sr_hold_set(sr8_hold_pio, sr8_hold_sm, pio_steps.length);
        sd_sr.reset.x_step = settings->steppers.step_invert.x;
#ifdef X_GANGED
        sd_sr.reset.m3_step = settings->steppers.step_invert.x;
#endif
        sd_sr.reset.y_step = settings->steppers.step_invert.y;
#ifdef Y_GANGED
        sd_sr.reset.m3_step = settings->steppers.step_invert.y;
#endif
        sd_sr.reset.z_step = settings->steppers.step_invert.z;
#ifdef Z_GANGED
        sd_sr.reset.m3_step = settings->steppers.step_invert.z;
#endif
#ifdef A_AXIS
        sd_sr.reset.m3_step = settings->steppers.step_invert.a;
#endif

#else // PIO step parameters init
        pio_steps.length = (uint32_t)(10.0f * (settings->steppers.pulse_microseconds - PIO_STEP_ADJ));
        pio_steps.delay = settings->steppers.pulse_delay_microseconds <= 0.8f
                            ? 2
                            : (uint32_t)(10.0f * (settings->steppers.pulse_delay_microseconds - PIO_STEP_ADJ));
#if N_GANGED
        axes_signals_t ganged_dir_invert = { .bits = settings->steppers.dir_invert.bits ^ settings->steppers.ganged_dir_invert.bits };
#endif

        gpio_set_outover(X_STEP_PIN, settings->steppers.step_invert.x ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
        gpio_set_outover(X_DIRECTION_PIN, settings->steppers.dir_invert.x ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
#if X_GANGED
        gpio_set_outover(X2_STEP_PIN, settings->steppers.step_invert.x ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
        gpio_set_outover(X2_DIRECTION_PIN, ganged_dir_invert.x ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
#endif

        gpio_set_outover(Y_STEP_PIN, settings->steppers.step_invert.y ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
        gpio_set_outover(Y_DIRECTION_PIN, settings->steppers.dir_invert.y ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
#if Y_GANGED
        gpio_set_outover(Y2_STEP_PIN, settings->steppers.step_invert.y ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
        gpio_set_outover(Y2_DIRECTION_PIN, ganged_dir_invert.y ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
#endif

        gpio_set_outover(Z_STEP_PIN, settings->steppers.step_invert.z ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
        gpio_set_outover(Z_DIRECTION_PIN, settings->steppers.dir_invert.z ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
#if Z_GANGED
        gpio_set_outover(Z2_STEP_PIN, settings->steppers.step_invert.z ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
        gpio_set_outover(Z2_DIRECTION_PIN, ganged_dir_invert.z ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
#endif

#ifdef A_AXIS
        gpio_set_outover(A_STEP_PIN, settings->steppers.step_invert.a ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
        gpio_set_outover(A_DIRECTION_PIN, settings->steppers.dir_invert.a ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
#endif
#ifdef B_AXIS
        gpio_set_outover(B_STEP_PIN, settings->steppers.step_invert.b ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
        gpio_set_outover(B_DIRECTION_PIN, settings->steppers.dir_invert.b ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
#endif
#ifdef C_AXIS
        gpio_set_outover(C_STEP_PIN, settings->steppers.step_invert.c ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
        gpio_set_outover(C_DIRECTION_PIN, settings->steppers.dir_invert.c ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
#endif

#endif // PIO step parameters init

        hal.stepper.go_idle(true);

        /***********************
         *  Input pins config  *
         ***********************/

        // Set the GPIO interrupt handler, the pin doesn't matter for now
        gpio_set_irq_enabled_with_callback(0, 0, false, gpio_int_handler);

        // Disable GPIO IRQ while initializing the input pins
        irq_set_enabled(IO_IRQ_BANK0, false);

        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
        input_signal_t *input;

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        do {

            input = &inputpin[--i];

            gpio_init(input->pin);
            if(!(input->group & PinGroup_Limit|PinGroup_LimitMax|PinGroup_AuxInput))
                gpio_set_irq_enabled(input->pin, GPIO_IRQ_ALL, false);

            switch(input->id) {

                case Input_EStop:
                    input->mode.pull_mode = settings->control_disable_pullup.e_stop ? PullMode_Down : PullMode_Up;
                    input->mode.inverted = control_fei.e_stop;
                    break;

                case Input_Reset:
                    input->mode.pull_mode = settings->control_disable_pullup.reset ? PullMode_Down : PullMode_Up;
                    input->mode.inverted = control_fei.reset;
                    break;

                case Input_FeedHold:
                    input->mode.pull_mode = settings->control_disable_pullup.feed_hold ? PullMode_Down : PullMode_Up;
                    input->mode.inverted = control_fei.feed_hold;
                    break;

                case Input_CycleStart:
                    input->mode.pull_mode = settings->control_disable_pullup.cycle_start ? PullMode_Down : PullMode_Up;
                    input->mode.inverted = control_fei.cycle_start;
                    break;

                case Input_LimitX:
                case Input_LimitX_2:
                case Input_LimitX_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.x ? PullMode_Down : PullMode_Up;
                    input->mode.inverted = limit_fei.x;
                    break;

                case Input_LimitY:
                case Input_LimitY_2:
                case Input_LimitY_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.y ? PullMode_Down : PullMode_Up;
                    input->mode.inverted = limit_fei.y;
                    break;

                case Input_LimitZ:
                case Input_LimitZ_2:
                case Input_LimitZ_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.z ? PullMode_Down : PullMode_Up;
                    input->mode.inverted = limit_fei.z;
                    break;

                case Input_LimitA:
                case Input_LimitA_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.a ? PullMode_Down : PullMode_Up;
                    input->mode.inverted = limit_fei.a;
                    break;

                case Input_LimitB:
                case Input_LimitB_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.b ? PullMode_Down : PullMode_Up;
                    input->mode.inverted = limit_fei.b;
                    break;

                case Input_LimitC:
                case Input_LimitC_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.c ? PullMode_Down : PullMode_Up;
                    input->mode.inverted = limit_fei.c;
                    break;

                case Input_SPIIRQ:
                    input->mode.pull_mode = PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Falling;
                    break;

                case Input_SpindleIndex:
                    input->mode.pull_mode = PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Falling;
                    break;

#if AUX_CONTROLS_ENABLED
  #if SAFETY_DOOR_BIT
                case Input_SafetyDoor:
                    safety_door = input;
                    input->mode.pull_mode = settings->control_disable_pullup.safety_door_ajar ? PullMode_Down : PullMode_Up;
                    input->mode.inverted = control_fei.safety_door_ajar;
                    input->mode.irq_mode = safety_door->invert ? IRQ_Mode_Low : IRQ_Mode_High;
                    break;
  #endif
#endif
#ifndef AUX_DEVICES
                case Input_Probe:
                    input->mode.pull_mode = settings->probe.disable_probe_pullup ? PullMode_Down : PullMode_Up;
                    input->mode.inverted = settings->probe.invert_probe_pin;
                    break;
  #ifdef MPG_MODE_PIN
                case Input_MPGSelect:
                    input->mode.pull_mode = PullMode_Up;
                    mpg_pin = input;
                    break;
  #endif
                case Input_I2CStrobe:
                    input->mode.pull_mode = PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;
#endif
                default:
                    break;
            }

            if(input->group & (PinGroup_Limit|PinGroup_LimitMax|PinGroup_Control))
                input->mode.irq_mode = input->mode.inverted ? IRQ_Mode_Falling : IRQ_Mode_Rising;

            gpio_set_pulls(input->pin, input->mode.pull_mode == PullMode_Up, input->mode.pull_mode != PullMode_Up);
            gpio_set_inover(input->pin, input->mode.inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

            if(!(input->group & (PinGroup_Limit|PinGroup_LimitMax|PinGroup_AuxInput)))
                pinEnableIRQ(input, input->mode.irq_mode);

#if PROBE_ENABLE
            if(input->id == Input_Probe)
                probeConfigure(false, false);
#endif
#if SAFETY_DOOR_BIT
            if(input->id == Input_SafetyDoor)
                input->active = DIGITAL_IN(input->pin);
#endif
            gpio_acknowledge_irq(input->pin, GPIO_IRQ_ALL);

        } while(i);

#if AUX_CONTROLS_ENABLED
        aux_ctrl_irq_enable(settings, aux_irq_handler);
#endif

        /*************************
         *  Output signals init  *
         *************************/
 
        output_signal_t *output;
        i = sizeof(outputpin) / sizeof(output_signal_t);

        do {
            output = &outputpin[--i];
            if(output->port == GPIO_OUTPUT)
              switch(output->id) {

                case Output_SpindleOn:
                    output->mode.inverted = settings->pwm_spindle.invert.on;
                    gpio_set_outover(output->pin, output->mode.inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
                    break;

                case Output_SpindleDir:
                    output->mode.inverted = settings->pwm_spindle.invert.ccw;
                    gpio_set_outover(output->pin, output->mode.inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
                    break;

                case Output_CoolantMist:
                    output->mode.inverted = settings->coolant.invert.mist;
                    gpio_set_outover(output->pin, output->mode.inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
                    break;

                case Output_CoolantFlood:
                    output->mode.inverted = settings->coolant.invert.flood;
                    gpio_set_outover(output->pin, output->mode.inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
                    break;

                default:
                    break;
            }
        } while(i);

        irq_set_enabled(IO_IRQ_BANK0, true); // Enable GPIO IRQ
    }
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    uint32_t i, id = 0;

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.id = id++;
        pin.pin = inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        //        pin.port = low_level ? NULL : (void *)port2char(inputpin[i].port);
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;
        pin.description = inputpin[i].description;

        pin_info(&pin, data);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.id = id++;
        pin.pin = outputpin[i].pin;
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.port = outputpin[i].port == GPIO_PIO ? (void *)"PIO" : (outputpin[i].port == GPIO_SR8 ? (void *)"SR8." : NULL);
        pin.description = outputpin[i].description;

        pin_info(&pin, data);
    };

    periph_signal_t *ppin = periph_pins;

    pin.port = NULL;

    if(ppin) do {
        pin.id = id++;
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description;

        pin_info(&pin, data);
    } while(ppin = ppin->next);
}

void registerPeriphPin (const periph_pin_t *pin)
{
    periph_signal_t *add_pin = malloc(sizeof(periph_signal_t));

    if(!add_pin)
        return;

    memcpy(&add_pin->pin, pin, sizeof(periph_pin_t));
    add_pin->next = NULL;

    if(periph_pins == NULL) {
        periph_pins = add_pin;
    } else {
        periph_signal_t *last = periph_pins;
        while (last->next)
            last = last->next;
        last->next = add_pin;
    }
}

void setPeriphPinDescription (const pin_function_t function, const pin_group_t group, const char *description)
{
    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        if(ppin->pin.function == function && ppin->pin.group == group) {
            ppin->pin.description = description;
            ppin = NULL;
        } else
            ppin = ppin->next;
    } while(ppin);
}

#ifdef NEOPIXELS_PIN

static void _write (void)
{
    // 50 us delay if busy? DMA?
    uint32_t *led = (uint32_t *)neopixel.leds;
//    uint64_t now = getElapsedMicros();
    
    while(pio_sm_get_tx_fifo_level(neop_pio, neop_sm) != 0);
//    while(getElapsedMicros() - now < 200);

    for(uint_fast16_t i = 0; i < neopixel.num_leds; i++)
        pio_sm_put_blocking(neop_pio, neop_sm, *led++);
}

static void neopixels_write (void)
{
    if(neopixel.num_leds > 1)
        _write();
}

static void neopixel_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(neopixel.num_leds && device < neopixel.num_leds) {

        uint8_t *led = &neopixel.leds[device * sizeof(uint32_t)] + 1;

        color = rgb_set_intensity(color, neopixel.intensity);

        if(mask.B)
            *led++ = color.B;
        else
            led++;     

        if(mask.R)
            *led++ = color.R;
        else
            led++;

        if(mask.G)
            *led = color.G;

       if(neopixel.num_leds == 1)
            _write();
    }
}

static void neopixel_out (uint16_t device, rgb_color_t color)
{
    neopixel_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

static inline rgb_color_t rp_rgb_1bpp_unpack (uint8_t *led, uint8_t intensity)
{
    rgb_color_t color = {0};

    if(intensity) {

        color.B = *led++;
        color.R = *led++; 
        color.G = *led; 

        color = rgb_reset_intensity(color, intensity);
    }

    return color;
}

static uint8_t neopixels_set_intensity (uint8_t intensity)
{
    uint8_t prev = neopixel.intensity;

    if(neopixel.intensity != intensity) {

        neopixel.intensity = intensity;

        if(neopixel.num_leds) {

            uint_fast16_t device = neopixel.num_leds;
            do {
                device--;
                rgb_color_t color = rp_rgb_1bpp_unpack(&neopixel.leds[device * sizeof(uint32_t)] + 1, prev);
                neopixel_out(device, color);
            } while(device);

            if(neopixel.num_leds > 1)
                _write();
        }
    }

    return prev;
}

#elif defined(LED_G_PIN)

static void board_led_out (uint16_t device, rgb_color_t color)
{
    if(device == 0)
        DIGITAL_OUT(LED_G_PIN, color.G != 0);
}

#elif WIFI_ENABLE || BLUETOOTH_ENABLE == 1

static void cyw43_led_on (void *data)
{
    if(net.value)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, data != NULL);
}

static void cyw43_led_out (uint16_t device, rgb_color_t color)
{
    if(device == 0) {
        if(net.value) // Network stack is up?
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, color.G != 0);
        else
            task_add_immediate(cyw43_led_on, color.G ? (void *)1 : NULL);
    }
}

#endif // NEOPIXELS_PIN

// Initializes MCU peripherals
static bool driver_setup (settings_t *settings)
{
    /*************************
     *  Output signals init  *
     *************************/

    for(uint_fast8_t i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        if(outputpin[i].port == GPIO_OUTPUT && outputpin[i].group != PinGroup_AuxOutputAnalog) {

            if(gpio_get_function(outputpin[i].pin) == GPIO_FUNC_PWM)
                continue;

            gpio_init(outputpin[i].pin);
            if(outputpin[i].id == PinGroup_StepperEnable || outputpin[i].id == Output_SdCardCS)
                DIGITAL_OUT(outputpin[i].pin, 1);

            gpio_set_dir_out_masked64(1ULL << outputpin[i].pin);

            if(outputpin[i].group == PinGroup_SpindlePWM)
                gpio_set_function(outputpin[i].pin, GPIO_FUNC_PWM);
        }
    }

#if SDCARD_ENABLE
    sdcard_init();
#elif LITTLEFS_ENABLE
    fs_macros_init();
#endif

#if MPG_ENABLE == 1
    gpio_init(MPG_MODE_PIN);
#endif

#if LITTLEFS_ENABLE
    fs_littlefs_mount(LITTLEFS_MOUNT_DIR , pico_littlefs_hal());
#endif

    IOInitDone = settings->version.id == 23;

    hal.settings_changed(settings, (settings_changed_flags_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

#if PPI_ENABLE
    ppi_init();
#endif

#if SPINDLE_ENCODER_ENABLE

    assert(pwm_gpio_to_channel(SPINDLE_PULSE_PIN) == PWM_CHAN_B);
    encoder_pwm = pwm_gpio_to_slice_num(SPINDLE_PULSE_PIN);
    
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_init(encoder_pwm, &cfg, false);
    gpio_set_function(SPINDLE_PULSE_PIN, GPIO_FUNC_PWM);

    pwm_set_enabled(encoder_pwm, true);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, rpm_int_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

#endif // SPINDLE_ENCODER_ENABLE

    return IOInitDone;
}

#if RP_MCU == 2040

static bool set_rtc_time (struct tm *time)
{
    datetime_t dt = {};
    dt.year = time->tm_year + 1900;
    dt.month = time->tm_mon + 1;
    dt.day = time->tm_mday;
    dt.hour = time->tm_hour;
    dt.min = time->tm_min;
    dt.sec = time->tm_sec;

    return (hal.driver_cap.rtc_set = rtc_set_datetime(&dt));
}

static bool get_rtc_time (struct tm *time)
{
    bool ok;
    datetime_t dt = {};

    if((ok = hal.driver_cap.rtc_set && rtc_get_datetime(&dt))) {
        time->tm_year = dt.year - 1900;
        time->tm_mon = dt.month - 1;
        time->tm_mday = dt.day;
        time->tm_hour = dt.hour;
        time->tm_min = dt.min;
        time->tm_sec = dt.sec;
    }

    return ok;
}

#endif

extern char __StackLimit, __bss_end__;

uint32_t get_free_mem (void)
{
    return &__StackLimit - &__bss_end__ - mallinfo().uordblks;
}

#if STEP_PORT == GPIO_PIO_1

static bool assign_step_sm (PIO *pio, uint *sm, uint32_t pin)
{
    static uint offset = 0;

    bool ok;

    if((ok = pio_claim_free_sm_and_add_program_for_gpio_range(&step_pulse_program, pio, sm, &offset, pin, 1, false)))
        step_pulse_program_init(*pio, *sm, offset, pin, 1, pio_clk);

    return ok;
}

#endif

//UF2 bootloader entry via command
#if USB_SERIAL_CDC

status_code_t enter_uf2 (sys_state_t state, char *args)
{

    hal.stream.write("[MSG:Warning: Entering UF2 Bootloader]" ASCII_EOL);
    hal.delay_ms(100, NULL);

    rom_reset_usb_boot(0, 0); // enter bootloader

    return Status_OK;
}

static void onReportOptions (bool newopt)
{
    if(!newopt)
        report_plugin("Bootloader Entry", "0.01");
}

#endif // USB_SERIAL_CDC

#ifdef SD_SHIFT_REGISTER

extern void sr16_init (sr_reg_t *reg);

void board_ports_init (void)
{
    xbar_t *se;
    uint8_t port = 0;

    sr16_init(&sr16);
/*
    if(hal.port.claim(Port_Analog, Port_Output, &port, "")) {
        se = hal.port.get_pin_info(Port_Analog, Port_Output, port);
        se->set_function(se, Output_StepperEnable);
        memcpy(&st_enable, se, sizeof(xbar_t));
    }*/
}
#endif

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: grblHAL is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    // Enable EEPROM and serial port here for grblHAL to be able to configure itself and report any errors

    // irq_set_exclusive_handler(-1, systick_handler);

    systick_hw->rvr = 999;
    systick_hw->cvr = 0;
#if RP_MCU == 2040
    systick_hw->csr = M0PLUS_SYST_CSR_TICKINT_BITS | M0PLUS_SYST_CSR_ENABLE_BITS;
#else
    systick_hw->csr = M33_SYST_CSR_TICKINT_BITS | M33_SYST_CSR_ENABLE_BITS;
#endif
#if RP_MCU == 2040
    hal.info = "RP2040";
#else
    hal.info = "RP2350";
#endif
    hal.driver_version = "250520";
    hal.driver_options = "SDK_" PICO_SDK_VERSION_STRING;
    hal.driver_url = GRBL_URL "/RP2040";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
#ifdef BOARD_URL
    hal.board_url = BOARD_URL;
#endif

    hal.driver_setup = driver_setup;
    hal.f_step_timer = 10000000;
    hal.f_mcu = clock_get_hz(clk_sys) / 1000000UL;
    hal.step_us_min = 1.0;
 
    pio_clk = (float)hal.f_mcu / 10.0f;

    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.get_free_mem = get_free_mem;
    hal.delay_ms = driver_delay;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
    hal.stepper.motor_iterator = motor_iterator;
#ifdef GANGING_ENABLED
    hal.stepper.get_ganged = getGangedAxes;
#endif
#ifdef SQUARING_ENABLED
    hal.stepper.disable_motors = StepperDisableMotors;
#endif
#if STEP_INJECT_ENABLE
    hal.stepper.output_step = stepperOutputStep;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

#ifdef PROBE_PIN
    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;
#endif

    hal.control.get_state = systemGetState;

#if I2C_STROBE_ENABLE || SPI_IRQ_BIT
    hal.irq_claim = irq_claim;
#endif
    hal.get_micros = getElapsedMicros;
    hal.get_elapsed_ticks = getElapsedTicks;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.enumerate_pins = enumeratePins;
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;

#if RP_MCU == 2040

    rtc_init();

    hal.driver_cap.rtc = On;
    hal.rtc.get_datetime = get_rtc_time;
    hal.rtc.set_datetime = set_rtc_time;

#endif

    serialRegisterStreams();

#if USB_SERIAL_CDC
    stream_connect(usb_serialInit());
#else
    if(!stream_connect_instance(SERIAL_STREAM, BAUD_RATE))
        while(true);
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#elif FLASH_ENABLE
    hal.nvs.type = NVS_Flash;
    hal.nvs.memcpy_from_flash = memcpy_from_flash;
    hal.nvs.memcpy_to_flash = memcpy_to_flash;
    hal.nvs.size_max = 4096; // 32K allocated
#else
    hal.nvs.type = NVS_None;
#endif

#if NVSDATA_BUFFER_ENABLE
    if(hal.nvs.type != NVS_None)
    	nvs_buffer_alloc(); // Reallocate memory block for NVS buffer
#endif

#if DRIVER_SPINDLE_ENABLE

 #if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR

        .ref_id = SPINDLE_PWM0,
#else
        .ref_id = SPINDLE_PWM0_NODIR,
#endif
        .config = spindleConfig,
        .set_state = spindleSetStateVariable,
        .get_state = spindleGetState,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindleSetSpeed,
  #if PPI_ENABLE
        .pulse_on = spindlePulseOn,
  #endif
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
            .pwm_invert = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR

            .direction = On
  #endif
        }
    };

 #else

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Basic,
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR

        .ref_id = SPINDLE_ONOFF0_DIR,
#else
        .ref_id = SPINDLE_ONOFF0,
#endif
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .cap = {
            .gpio_controlled = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR

            .direction = On
  #endif
        }
    };

 #endif

    spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME);

#endif // DRIVER_SPINDLE_ENABLE

// driver capabilities

#ifndef RESET_PIN
    hal.signals_cap.reset = Off;
#elif ESTOP_ENABLE
    hal.signals_cap.e_stop = On;
    hal.signals_cap.reset = Off;
#endif
    hal.limits_cap = get_limits_cap();
    hal.home_cap = get_home_cap();
#if defined(COOLANT_FLOOD_PIN) || OUT_SHIFT_REGISTER
    hal.coolant_cap.flood = On;
#endif
#if defined(COOLANT_MIST_PIN) || OUT_SHIFT_REGISTER
    hal.coolant_cap.mist = On;
#endif
#if SPINDLE_ENCODER_ENABLE
    hal.driver_cap.spindle_encoder = On;
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
#ifdef PROBE_PIN
    hal.driver_cap.probe_pull_up = On;
#endif

    uint32_t i;
    input_signal_t *input;
    output_signal_t *output;

    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0}, aux_inputs_analog = {0}, aux_outputs_analog = {0};

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        input = &inputpin[i];
        input->mode.input = input->cap.input = On;
        input->mode.pull_mode = input->cap.pull_mode = PullMode_Up;
        irq_pins[input->pin] = input;
        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;
            input->user_port = aux_inputs.n_pins++;
            input->id = Input_Aux0 + input->user_port;
            input->cap.irq_mode = IRQ_Mode_All;
            input->cap.debounce = hal.driver_cap.software_debounce;
#if AUX_CONTROLS_ENABLED
            aux_ctrl_t *aux_remap;
            if((aux_remap = aux_ctrl_remap_explicit(NULL, input->pin, input->user_port, input))) {
                if(aux_remap->function == Input_Probe)
                    aux_remap->irq_mode = IRQ_Mode_Change;
            }
#endif
        } else if(input->group & (PinGroup_Limit|PinGroup_LimitMax)) {
//            input->cap.debounce = hal.driver_cap.software_debounce;
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            limit_inputs.n_pins++;
        } else if(input->group == PinGroup_Control) {
            input->cap.debounce = hal.driver_cap.software_debounce;
#if !AUX_CONTROLS_ENABLED && SAFETY_DOOR_BIT
            if(input->id == Input_SafetyDoor)
                safety_door = input;
#endif
        }
    }

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        output->mode.output = On;
        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            output->id = Output_Aux0 + aux_outputs.n_pins;
#if AUX_CONTROLS
            aux_out_remap_explicit((void *)((uint32_t)output->port), output->pin, aux_outputs.n_pins, output);
#endif
            aux_outputs.n_pins++;
        } else if(output->group == PinGroup_AuxOutputAnalog) {
            if(aux_outputs_analog.pins.outputs == NULL)
                aux_outputs_analog.pins.outputs = output;
            output->mode.analog = On;
            output->id = Output_Analog_Aux0 + aux_outputs_analog.n_pins++;
        }
    }

    if(aux_inputs.n_pins || aux_outputs.n_pins)
        ioports_init(&aux_inputs, &aux_outputs);

    if(aux_outputs_analog.n_pins)
        ioports_init_analog(&aux_inputs_analog, &aux_outputs_analog);

    io_expanders_init();

#if AUX_CONTROLS_ENABLED
    aux_ctrl_claim_ports(aux_claim_explicit, NULL);
#elif defined(SAFETY_DOOR_PIN)
    hal.signals_cap.safety_door = On;
#endif

#if AUX_CONTROLS
    aux_ctrl_claim_out_ports(aux_out_claim_explicit, NULL);
#endif

#if USB_SERIAL_CDC

// Register $UF2 bootloader command

    static const sys_command_t boot_command_list[] = {
        {"UF2", enter_uf2, { .noargs = On }, { .str = "enter UF2 bootloader" } }
    };

    static sys_commands_t boot_commands = {
        .n_commands = sizeof(boot_command_list) / sizeof(sys_command_t),
        .commands = boot_command_list
    };

    grbl.on_report_options = onReportOptions;

    system_register_commands(&boot_commands);

#endif // USB_SERIAL_CDC

#if WIFI_ENABLE
    wifi_init();
#endif

#if ETHERNET_ENABLE
    enet_init();
#endif

#if BLUETOOTH_ENABLE == 1
    bluetooth_init_local();
#endif

// Stepper init

    uint pio_offset;
#if WIFI_ENABLE || BLUETOOTH_ENABLE == 1
    pio_sm_claim(pio1, 0); // Reserve PIO state machine for cyw43 driver.
#endif

    stepper_timer_sm = pio_claim_unused_sm(pio1, false);
    stepper_timer_sm_offset = pio_add_program(pio1, &stepper_timer_program);
    stepper_timer_program_init(pio1, stepper_timer_sm, stepper_timer_sm_offset, pio_clk); // 10MHz
    irq_set_exclusive_handler(PIO1_IRQ_0, stepper_int_handler);
    irq_set_priority(PIO1_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);

#if STEP_PORT == GPIO_PIO_1
    assign_step_sm(&x_step_pio, &x_step_sm, X_STEP_PIN);
    assign_step_sm(&y_step_pio, &y_step_sm, Y_STEP_PIN);
    assign_step_sm(&z_step_pio, &z_step_sm, Z_STEP_PIN);

#if N_ABC_MOTORS

#if WIFI_ENABLE && N_ABC_MOTORS > 2
#error "Max number of motors with WIFI_ENABLE is 5"
#endif

#ifdef X2_STEP_PIN
    assign_step_sm(&x2_step_pio, &x2_step_sm, X2_STEP_PIN);
#endif
#ifdef Y2_STEP_PIN
    assign_step_sm(&y2_step_pio, &y2_step_sm, Y2_STEP_PIN);
#endif
#ifdef Z2_STEP_PIN
    assign_step_sm(&z2_step_pio, &z2_step_sm, Z2_STEP_PIN);
#endif
#ifdef A_STEP_PIN
    assign_step_sm(&a_step_pio, &a_step_sm, A_STEP_PIN);
#endif
#ifdef B_STEP_PIN
    assign_step_sm(&b_step_pio, &b_step_sm, B_STEP_PIN);
#endif
#ifdef C_STEP_PIN
    assign_step_sm(&c_step_pio, &c_step_sm, C_STEP_PIN);
#endif

#endif // N_ABC_MOTORS

#elif STEP_PORT == GPIO_PIO

if(pio_claim_free_sm_and_add_program_for_gpio_range(&step_pulse_program, &step_pio, &step_sm, &pio_offset, STEP_PINS_BASE, N_AXIS + N_GANGED, false))
   step_pulse_program_init(step_pio, step_sm, pio_offset, STEP_PINS_BASE, N_AXIS + N_GANGED, pio_clk);

#elif STEP_PORT == GPIO_SR8

    pio_offset = pio_add_program(pio0, &step_dir_sr4_program);
    step_dir_sr4_program_init(pio0, 0, pio_offset, SD_SR_DATA_PIN, SD_SR_SCK_PIN);

    pio_offset = pio_add_program(pio0, &sr_delay_program);
    sr_delay_program_init(pio0, 1, pio_offset, pio_clk - 0.37f);

    pio_offset = pio_add_program(pio0, &sr_hold_program);
    sr_hold_program_init(pio0, 2, pio_offset, pio_clk - 0.37f);

    pio_claim_sm_mask(pio0, 0b1111); // claim all state machines, no room for more programs
sr8_sm = 0;
sr8_delay_sm = 1;
sr8_hold_sm = 2;
sr8_pio = sr8_delay_pio = sr8_hold_pio = pio0;
/*
    if(pio_claim_free_sm_and_add_program_for_gpio_range(&step_dir_sr4_program, &sr8_pio, &sr8_sm, &pio_offset, SD_SR_DATA_PIN, 3, false)) {

        step_dir_sr4_program_init(sr8_pio, sr8_sm, pio_offset, SD_SR_DATA_PIN, SD_SR_SCK_PIN);

        if(pio_claim_free_sm_and_add_program(&sr_delay_program, &sr8_delay_pio, &sr8_delay_sm, &pio_offset))
            sr_delay_program_init(sr8_delay_pio, sr8_delay_sm, pio_offset, 11.65f);

        if(pio_claim_free_sm_and_add_program(&sr_hold_program, &sr8_hold_pio, &sr8_hold_sm, &pio_offset))
            sr_hold_program_init(sr8_hold_pio, sr8_hold_sm, pio_offset, 11.65f);
    }
*/
#endif

#if OUT_SHIFT_REGISTER
    if(pio_claim_free_sm_and_add_program_for_gpio_range(&out_sr16_program, &sr16.pio, &sr16.sm, &pio_offset, OUT_SR_DATA_PIN, 3, false)) {
        out_sr16_program_init(sr16.pio, sr16.sm, pio_offset, OUT_SR_DATA_PIN, OUT_SR_SCK_PIN);
  #if SPI_RST_PORT == EXPANDER_PORT
        spi_reset_out(1);
  #endif
    }
#endif

#ifdef NEOPIXELS_PIN

    if(pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &neop_pio, &neop_sm, &pio_offset, NEOPIXELS_PIN, 1, false)) {

        ws2812_program_init(neop_pio, neop_sm, pio_offset, NEOPIXELS_PIN, 800000, false);

        hal.rgb0.out = neopixel_out;
        hal.rgb0.out_masked = neopixel_out_masked;
        hal.rgb0.set_intensity = neopixels_set_intensity;
        hal.rgb0.write = neopixels_write;
        hal.rgb0.num_devices = NEOPIXELS_NUM;
        hal.rgb0.flags = (rgb_properties_t){ .is_blocking = On, .is_strip = On };
        hal.rgb0.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };

        const periph_pin_t neopin = {
            .group = PinGroup_LED,
            .function = Output_LED_Adressable,
            .pin = NEOPIXELS_PIN
        };

        registerPeriphPin(&neopin);
    } // else report unavailable?

#elif defined(LED_G_PIN)

    hal.rgb0.out = board_led_out;
    hal.rgb0.num_devices = 1;
    hal.rgb0.cap = (rgb_color_t){ .R = 0, .G = 1, .B = 0 };

#elif WIFI_ENABLE || BLUETOOTH_ENABLE == 1

    hal.rgb0.out = cyw43_led_out;
    hal.rgb0.num_devices = 1;
    hal.rgb0.cap = (rgb_color_t){ .R = 0, .G = 1, .B = 0 };

    hal.rgb0.out(0, hal.rgb0.cap);

#endif // NEOPIXELS_PIN

#ifdef HAS_BOARD_INIT
    board_init();
#endif

#include "grbl/plugins_init.h"

#if MPG_ENABLE == 1
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, NULL);
    if(hal.driver_cap.mpg_mode)
        task_run_on_startup(mpg_enable, NULL);
#elif MPG_ENABLE == 2
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, stream_mpg_check_enable);
#endif

#if WIFI_ENABLE || BLUETOOTH_ENABLE == 1
    pio_sm_unclaim(pio1, 0);  // Release PIO state machine for cyw43 driver.
#endif

    //  debounceAlarmPool = alarm_pool_create(DEBOUNCE_ALARM_HW_TIMER, DEBOUNCE_ALARM_MAX_TIMER);

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 10;
}

/* interrupt handlers */

// Main stepper driver
static void __not_in_flash_func(stepper_int_handler)(void)
{
    hal.stepper.interrupt_callback();

    stepper_timer_irq_clear(pio1);
}

#if PPI_ENABLE

// PPI timer interrupt handler
static void __not_in_flash_func(PPI_TIMER_IRQHandler)(void)
{
    PPI_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag;

    spindle_off(ppi_spindle);
}

#endif

#if SPINDLE_ENCODER_ENABLE

static void __not_in_flash_func(rpm_int_handler)(void)
{
    encoder_ovf += 0x10000;
    pwm_clear_irq(encoder_pwm);
}

#endif // SPINDLE_ENCODER_ENABLE

void pin_debounce (void *pin)
{
    input_signal_t *input = (input_signal_t *)pin;

#ifdef SAFETY_DOOR_PIN
    if(input->id == Input_SafetyDoor)
        debounce.safety_door = Off;
#endif

    if(input->mode.irq_mode == IRQ_Mode_Change ||
        (DIGITAL_IN(input->pin) ^ input->mode.inverted) == (input->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1)) {

        switch(input->group) {

            case PinGroup_Control:
                hal.control.interrupt_callback(systemGetState());
                break;

            case PinGroup_Limit:
            case PinGroup_LimitMax:
                {
                    limit_signals_t state = limitsGetState();
                    if(limit_signals_merge(state).value)
                        hal.limits.interrupt_callback(state);
                }
                break;

            case PinGroup_AuxInput:
                ioports_event(input);
                break;

            default:
                break;
        }
    }

    pinEnableIRQ(input, (pin_irq_mode_t)input->mode.irq_mode); // Reenable pin interrupt
}

// GPIO Interrupt handler
// TODO: bypass the Pico library interrupt handler.
void __not_in_flash_func(gpio_int_handler)(uint pin, uint32_t events)
{
    input_signal_t *input;

    if((input = irq_pins[pin])) {

    #if SPI_IRQ_BIT
        if(input->id == Input_SPIIRQ && spi_irq.callback)
            spi_irq.callback(0, DIGITAL_IN(input->pin) == 0);
        else
    #endif
        if(input->mode.debounce && task_add_delayed(pin_debounce, input, 40)) {
            gpio_set_irq_enabled(pin, GPIO_IRQ_ALL, false);
#if SAFETY_DOOR_ENABLE
            if(input->id == Input_SafetyDoor)
                debounce.safety_door = input->mode.debounce;
#endif
        } else switch (input->group) {

            case PinGroup_Probe:
                if(input->id == Input_Probe) {
                    if(probe.is_probing)
                        probe.triggered = On;
                    else {
                        control_signals_t signals = {0};
                        signals.probe_triggered = On;
                        hal.control.interrupt_callback(signals);
                    }
//                    if(task_add_delayed(probe_irq_enable, input, 40))
//                        gpio_set_irq_enabled(gpio, GPIO_IRQ_ALL, false);
                }
                break;

            case PinGroup_Control:
                hal.control.interrupt_callback(systemGetState());
                break;

            case PinGroup_Limit:
            case PinGroup_LimitMax:
                hal.limits.interrupt_callback(limitsGetState());
                break;

            case PinGroup_AuxInput:
                ioports_event(input);
                break;

#if SPINDLE_ENCODER_ENABLE
            case PinGroup_SpindleIndex:
                uint32_t rpm_count = encoder_ovf | pwm_get_counter(encoder_pwm);
                spindle_encoder.timer.last_index = (uint32_t)(get_absolute_time() - encoder_started);

                if(spindle_encoder.counter.index_count && (uint16_t)(rpm_count - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
                    spindle_encoder.error_count++;

                spindle_encoder.counter.last_index = rpm_count;
                spindle_encoder.counter.index_count++;
                break;
#endif

    #ifndef AUX_DEVICES
      #ifdef I2C_STROBE_PIN
            case PinGroup_I2C:
                if(input->id == Input_I2CStrobe && i2c_strobe.callback)
                    i2c_strobe.callback(0, DIGITAL_IN(input->pin) == 0);
                break;
      #endif
      #if MPG_ENABLE == 1
            case PinGroup_MPG:
                task_add_immediate(mpg_select, NULL);
                break;
      #endif
    #endif // !AUX_DEVICES
            default:
                break;
        }
    }
}

// Interrupt handler for 1 ms interval timer
void __not_in_flash_func(isr_systick)(void)
{
    elapsed_ticks++;

#if SDCARD_ENABLE
    static uint32_t fatfs_ticks = 10;
    if(!(--fatfs_ticks)) {
        disk_timerproc();
        fatfs_ticks = 10;
    }
#endif
}
