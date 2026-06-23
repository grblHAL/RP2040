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
#include "grbl/core_handlers.h"

static control_signals_get_state_ptr hal_control_get_state;
static stepper_status_t stepper_status = {};

typedef struct {
    uint8_t n_pins;
    struct {
        uint8_t axis;
        bool secondary;
        input_signal_t *input;
    } motor[N_ABC_MOTORS + 3];
} motor_pins_t;

static motor_pins_t fault_signals = {};

void motor_fault_add_pin (input_signal_t *input, xbar_t *pin)
{
    fault_signals.motor[fault_signals.n_pins].input = input;
    fault_signals.motor[fault_signals.n_pins].axis = xbar_fault_pin_to_axis(pin->function);
    fault_signals.motor[fault_signals.n_pins++].secondary = pin->function >= Input_MotorFaultX2;
}

static void update_motor_fault_status (void)
{
    uint_fast8_t idx;

    stepper_status.fault.state = 0;

    if(!settings.motor_fault_enable.mask)
        return;

    for(idx = 0; idx < fault_signals.n_pins; idx++) {

        if(bit_istrue(settings.motor_fault_enable.mask, bit(fault_signals.motor[idx].axis))) {

            input_signal_t *input = fault_signals.motor[idx].input;
            bool inverted = bit_istrue(settings.motor_fault_invert.mask, bit(fault_signals.motor[idx].axis));

            if((DIGITAL_IN(input->pin) != 0) ^ inverted)
                xbar_stepper_state_set(&stepper_status.fault, fault_signals.motor[idx].axis, fault_signals.motor[idx].secondary);
        }
    }
}

static void motor_fault_irq_handler (uint8_t port, bool state)
{
    (void)port;
    (void)state;

    update_motor_fault_status();

    if(stepper_status.fault.state && !(state_get() & (STATE_ALARM|STATE_ESTOP))) {
        control_signals_t signals = hal_control_get_state();
        signals.motor_fault = On;
        hal.control.interrupt_callback(signals);
    }
}

static stepper_status_t getDriverStatus (bool reset)
{
    if(reset)
        stepper_status.fault.state = 0;
    else
        update_motor_fault_status();

    return stepper_status;
}

static control_signals_t getControlState (void)
{
    control_signals_t state = hal_control_get_state();

    update_motor_fault_status();
    state.motor_fault = stepper_status.fault.state != 0;

    return state;
}

static void motor_fault_init (void *arg)
{
    uint_fast8_t idx;

    if(!fault_signals.n_pins)
        return;

    hal.signals_cap.motor_fault = On;
    hal.stepper.status = getDriverStatus;

    hal_control_get_state = hal.control.get_state;
    hal.control.get_state = getControlState;

    for(idx = 0; idx < fault_signals.n_pins; idx++) {
        input_signal_t *input = fault_signals.motor[idx].input;

        input->mode.irq_mode = IRQ_Mode_Change;
        input->interrupt_callback = motor_fault_irq_handler;
        pinEnableIRQ(input, input->mode.irq_mode);
    }

    update_motor_fault_status();
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
