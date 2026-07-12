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
#include "grbl/state_machine.h"

#define MOTOR_FAULT_ARM_DELAY_MS 1000

static driver_setup_ptr driver_setup;
static control_signals_get_state_ptr hal_control_get_state;
static stepper_status_t stepper_status = {};
static volatile bool motor_fault_armed = false;
static bool motor_fault_initialized = false;
static bool motor_fault_irq_installed = false;
typedef struct {
    uint8_t n_pins;
    struct {
        uint8_t axis;
        bool secondary;
        uint8_t pin;
    } motor[N_ABC_MOTORS + 3];
} motor_pins_t;

static motor_pins_t fault_signals = {};
#if NUM_BANK0_GPIOS > 32
static const uint64_t fault_pin_mask =
    (1ull << X_MOTOR_FAULT_PIN) |
    (1ull << Y_MOTOR_FAULT_PIN) |
    (1ull << Z_MOTOR_FAULT_PIN)
#ifdef M3_MOTOR_FAULT_PIN
    | (1ull << M3_MOTOR_FAULT_PIN)
#endif
#ifdef M4_MOTOR_FAULT_PIN
    | (1ull << M4_MOTOR_FAULT_PIN)
#endif
    ;
#else
static const uint32_t fault_pin_mask =
    (1u << X_MOTOR_FAULT_PIN) |
    (1u << Y_MOTOR_FAULT_PIN) |
    (1u << Z_MOTOR_FAULT_PIN)
#ifdef M3_MOTOR_FAULT_PIN
    | (1u << M3_MOTOR_FAULT_PIN)
#endif
#ifdef M4_MOTOR_FAULT_PIN
    | (1u << M4_MOTOR_FAULT_PIN)
#endif
    ;
#endif

void motor_fault_add_pin (input_signal_t *input, xbar_t *pin)
{
    (void)input;
    (void)pin;
}

static void motor_fault_register_pin (uint8_t axis, bool secondary, uint8_t pin)
{
    fault_signals.motor[fault_signals.n_pins].axis = axis;
    fault_signals.motor[fault_signals.n_pins].secondary = secondary;
    fault_signals.motor[fault_signals.n_pins++].pin = pin;
}

static void motor_fault_populate_pins (void)
{
    if(fault_signals.n_pins)
        return;

    motor_fault_register_pin(X_AXIS, false, X_MOTOR_FAULT_PIN);
    motor_fault_register_pin(Y_AXIS, false, Y_MOTOR_FAULT_PIN);
    motor_fault_register_pin(Z_AXIS, false, Z_MOTOR_FAULT_PIN);

#ifdef M3_MOTOR_FAULT_PIN
  #ifdef A_AXIS
    motor_fault_register_pin(A_AXIS, false, M3_MOTOR_FAULT_PIN);
  #elif X_GANGED
    motor_fault_register_pin(X_AXIS, true, M3_MOTOR_FAULT_PIN);
  #elif Y_GANGED
    motor_fault_register_pin(Y_AXIS, true, M3_MOTOR_FAULT_PIN);
  #elif Z_GANGED
    motor_fault_register_pin(Z_AXIS, true, M3_MOTOR_FAULT_PIN);
  #endif
#endif
#ifdef M4_MOTOR_FAULT_PIN
  #ifdef B_AXIS
    motor_fault_register_pin(B_AXIS, false, M4_MOTOR_FAULT_PIN);
  #elif X_GANGED
    motor_fault_register_pin(X_AXIS, true, M4_MOTOR_FAULT_PIN);
  #elif Y_GANGED
    motor_fault_register_pin(Y_AXIS, true, M4_MOTOR_FAULT_PIN);
  #elif Z_GANGED
    motor_fault_register_pin(Z_AXIS, true, M4_MOTOR_FAULT_PIN);
  #endif
#endif
}

static void onMotorFaultIRQ (void *data)
{
    uint_fast8_t idx;

    (void)data;

    stepper_status.fault.state = 0;

    if(!motor_fault_armed || !settings.motor_fault_enable.mask)
        return;

    for(idx = 0; idx < fault_signals.n_pins; idx++) {
        if(bit_istrue(settings.motor_fault_enable.mask, bit(fault_signals.motor[idx].axis))) {
            bool inverted = bit_istrue(settings.motor_fault_invert.mask, bit(fault_signals.motor[idx].axis));

            if((DIGITAL_IN(fault_signals.motor[idx].pin) != 0) ^ inverted)
                xbar_stepper_state_set(&stepper_status.fault, fault_signals.motor[idx].axis, fault_signals.motor[idx].secondary);
        }
    }

    if(stepper_status.fault.state && !(state_get() & (STATE_ALARM|STATE_ESTOP))) {
        control_signals_t signals = hal_control_get_state();
        signals.motor_fault = On;
        hal.control.interrupt_callback(signals);
    }
}

static void motor_fault_irq_handler (void)
{
    uint_fast8_t idx;
    bool changed = false;

    for(idx = 0; idx < fault_signals.n_pins; idx++) {
        uint32_t events = gpio_get_irq_event_mask(fault_signals.motor[idx].pin);

        if(events) {
            gpio_acknowledge_irq(fault_signals.motor[idx].pin, events);
            changed = true;
        }
    }

    if(!changed)
        return;

    onMotorFaultIRQ(NULL);
}

static void arm_motor_fault_monitor (void *data)
{
    uint_fast8_t idx;

    (void)data;

    motor_fault_armed = true;

    for(idx = 0; idx < fault_signals.n_pins; idx++)
        gpio_set_irq_enabled(fault_signals.motor[idx].pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    onMotorFaultIRQ(NULL);
}

static stepper_status_t getDriverStatus (bool reset)
{
    if(reset)
        stepper_status.fault.state = 0;

    return stepper_status;
}

static control_signals_t getControlState (void)
{
    control_signals_t state = hal_control_get_state();

    state.motor_fault = stepper_status.fault.state != 0;

    return state;
}

static void motor_fault_init (void)
{
    uint_fast8_t idx;

    motor_fault_populate_pins();

    if(motor_fault_initialized)
        return;

    for(idx = 0; idx < fault_signals.n_pins; idx++) {
        gpio_init(fault_signals.motor[idx].pin);
        gpio_set_dir(fault_signals.motor[idx].pin, GPIO_IN);
        gpio_set_irq_enabled(fault_signals.motor[idx].pin, GPIO_IRQ_ALL, false);
    }

    motor_fault_initialized = true;
}

static bool driverSetup (settings_t *settings)
{
    if(!driver_setup(settings))
        return false;

    motor_fault_init();

    if((hal.signals_cap.motor_fault = !!settings->motor_fault_enable.value && fault_signals.n_pins)) {

        if(!motor_fault_irq_installed) {
#if NUM_BANK0_GPIOS > 32
            gpio_add_raw_irq_handler_masked64(fault_pin_mask, motor_fault_irq_handler);
#else
            gpio_add_raw_irq_handler_masked(fault_pin_mask, motor_fault_irq_handler);
#endif
            irq_set_enabled(IO_IRQ_BANK0, true);
            motor_fault_irq_installed = true;
        }

        stepper_status.fault.state = 0;
        motor_fault_armed = false;

        hal.stepper.status = getDriverStatus;

        hal_control_get_state = hal.control.get_state;
        hal.control.get_state = getControlState;

        task_add_delayed(arm_motor_fault_monitor, NULL, MOTOR_FAULT_ARM_DELAY_MS);
    }

    return true;
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

    driver_setup = hal.driver_setup;
    hal.driver_setup = driverSetup;

    task_run_on_startup(sr_oe_init, NULL);
    task_run_on_startup(home_indicator_init, NULL);
}

#endif // defined(HAS_BOARD_INIT) && defined(BOARD_SLB_LITE)
