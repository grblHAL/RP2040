/*
  slblite.c - board driver for Sienci Labs SLB-Lite

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

#include "driver.h"

#if defined(HAS_BOARD_INIT) && defined(BOARD_SLB_LITE)

#include "grbl/task.h"
#include "grbl/system.h"
#include "grbl/state_machine.h"
#include "grbl/core_handlers.h"

#define MOTOR_FAULT_ARM_DELAY_MS 1000

static stepper_status_t stepper_status = {};
static const bool motor_fault_invert = true;
static volatile bool motor_fault_armed = false;
static const uint8_t fault_pins[] = {
    SLBLITE_X_MOTOR_FAULT_PIN,
    SLBLITE_Y_MOTOR_FAULT_PIN,
    SLBLITE_Z_MOTOR_FAULT_PIN,
#ifdef SLBLITE_M3_MOTOR_FAULT_PIN
    SLBLITE_M3_MOTOR_FAULT_PIN,
#endif
#ifdef SLBLITE_M4_MOTOR_FAULT_PIN
    SLBLITE_M4_MOTOR_FAULT_PIN,
#endif
};
#if NUM_BANK0_GPIOS > 32
static const uint64_t fault_pin_mask =
    (1ull << SLBLITE_X_MOTOR_FAULT_PIN) |
    (1ull << SLBLITE_Y_MOTOR_FAULT_PIN) |
    (1ull << SLBLITE_Z_MOTOR_FAULT_PIN)
#ifdef SLBLITE_M3_MOTOR_FAULT_PIN
    | (1ull << SLBLITE_M3_MOTOR_FAULT_PIN)
#endif
#ifdef SLBLITE_M4_MOTOR_FAULT_PIN
    | (1ull << SLBLITE_M4_MOTOR_FAULT_PIN)
#endif
    ;
#else
static const uint32_t fault_pin_mask =
    (1u << SLBLITE_X_MOTOR_FAULT_PIN) |
    (1u << SLBLITE_Y_MOTOR_FAULT_PIN) |
    (1u << SLBLITE_Z_MOTOR_FAULT_PIN)
#ifdef SLBLITE_M3_MOTOR_FAULT_PIN
    | (1u << SLBLITE_M3_MOTOR_FAULT_PIN)
#endif
#ifdef SLBLITE_M4_MOTOR_FAULT_PIN
    | (1u << SLBLITE_M4_MOTOR_FAULT_PIN)
#endif
    ;
#endif

void motor_fault_add_pin (input_signal_t *input, xbar_t *pin)
{
    (void)input;
    (void)pin;
}

static bool motor_fault_active (uint8_t pin)
{
    return (DIGITAL_IN(pin) != 0) ^ motor_fault_invert;
}

static void update_motor_fault_status (void)
{
    uint_fast8_t idx;

    stepper_status.fault.state = 0;

    if(!motor_fault_armed)
        return;

    for(idx = 0; idx < sizeof(fault_pins) / sizeof(fault_pins[0]); idx++) {
        if(motor_fault_active(fault_pins[idx])) {
            stepper_status.fault.state = 1;
            break;
        }
    }
}

static void motor_fault_irq_handler (void)
{
    uint_fast8_t idx;
    bool changed = false;

    for(idx = 0; idx < sizeof(fault_pins) / sizeof(fault_pins[0]); idx++) {
        uint32_t events = gpio_get_irq_event_mask(fault_pins[idx]);

        if(events) {
            gpio_acknowledge_irq(fault_pins[idx], events);
            changed = true;
        }
    }

    if(!changed)
        return;

    update_motor_fault_status();

    if(stepper_status.fault.state && !(state_get() & (STATE_ALARM|STATE_ESTOP))) {
        system_set_exec_alarm(Alarm_MotorFault);
    }
}

static void arm_motor_fault_monitor (void *data)
{
    uint_fast8_t idx;

    (void)data;

    motor_fault_armed = true;

    for(idx = 0; idx < sizeof(fault_pins) / sizeof(fault_pins[0]); idx++)
        gpio_set_irq_enabled(fault_pins[idx], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    update_motor_fault_status();

    if(stepper_status.fault.state && !(state_get() & (STATE_ALARM|STATE_ESTOP)))
        system_set_exec_alarm(Alarm_MotorFault);
}

static stepper_status_t getDriverStatus (bool reset)
{
    if(reset)
        stepper_status.fault.state = 0;
    else if(motor_fault_armed)
        update_motor_fault_status();

    return stepper_status;
}

static void motor_fault_init (void *arg)
{
    uint_fast8_t idx;

    hal.stepper.status = getDriverStatus;
    motor_fault_armed = false;

    for(idx = 0; idx < sizeof(fault_pins) / sizeof(fault_pins[0]); idx++) {
        gpio_init(fault_pins[idx]);
        gpio_set_dir(fault_pins[idx], GPIO_IN);
        gpio_set_irq_enabled(fault_pins[idx], GPIO_IRQ_ALL, false);
    }

#if NUM_BANK0_GPIOS > 32
    gpio_add_raw_irq_handler_masked64(fault_pin_mask, motor_fault_irq_handler);
#else
    gpio_add_raw_irq_handler_masked(fault_pin_mask, motor_fault_irq_handler);
#endif
    irq_set_enabled(IO_IRQ_BANK0, true);

    task_add_delayed(arm_motor_fault_monitor, NULL, MOTOR_FAULT_ARM_DELAY_MS);
}

// Shift register Output Enable — pulled LOW at boot to enable 74HCT595 outputs.
// task_run_on_startup pattern from Ooznest example:
// https://github.com/Ooznest/ESP32/blob/master/main/boards/ooznest_cnc.c
static void sr_oe_init (void *arg)
{
    gpio_init(OUT_SR_OE_PIN);
    gpio_set_dir(OUT_SR_OE_PIN, GPIO_OUT);
    gpio_put(OUT_SR_OE_PIN, 0);
}

// Homing indicator outputs — driven by on_homing_rate_set / on_homing_completed.
static axes_signals_t homing_axes = {0};
static on_homing_rate_set_ptr saved_on_homing_rate_set;
static on_homing_completed_ptr saved_on_homing_completed;

static void onHomingRateSet (axes_signals_t axes, coord_data_t *feedrate, homing_mode_t mode)
{
    homing_axes = axes;

    gpio_put(HOME_INDICATOR_Z_PIN, axes.z ? 0 : 1);
    gpio_put(HOME_INDICATOR_XYZA_PIN, (axes.x || axes.y || axes.a) ? 0 : 1);

    if(saved_on_homing_rate_set)
        saved_on_homing_rate_set(axes, feedrate, mode);
}

static void onHomingCompleted (axes_signals_t cycle, bool success)
{
    homing_axes.mask = 0;

    gpio_put(HOME_INDICATOR_Z_PIN, 1);
    gpio_put(HOME_INDICATOR_XYZA_PIN, 1);

    if(saved_on_homing_completed)
        saved_on_homing_completed(cycle, success);
}

static void home_indicator_init (void *arg)
{
    gpio_put(HOME_INDICATOR_Z_PIN, 1);
    gpio_put(HOME_INDICATOR_XYZA_PIN, 1);
}

void board_init (void)
{
    gpio_init(HOME_INDICATOR_Z_PIN);
    gpio_set_dir(HOME_INDICATOR_Z_PIN, GPIO_OUT);
    gpio_init(HOME_INDICATOR_XYZA_PIN);
    gpio_set_dir(HOME_INDICATOR_XYZA_PIN, GPIO_OUT);

    saved_on_homing_rate_set = grbl.on_homing_rate_set;
    grbl.on_homing_rate_set = onHomingRateSet;

    saved_on_homing_completed = grbl.on_homing_completed;
    grbl.on_homing_completed = onHomingCompleted;

    task_run_on_startup(sr_oe_init, NULL);
    task_run_on_startup(motor_fault_init, NULL);
    task_run_on_startup(home_indicator_init, NULL);
}

#endif // defined(HAS_BOARD_INIT) && defined(BOARD_SLB_LITE)
