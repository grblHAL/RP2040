/*
  flexihal2350.c - init code for flexihal2350

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io
  Copyright (c) 2025 Expatria Technologies Inc.
  Copyright (c) 2026 Mitchell Grams
  
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

#if defined(BOARD_FLEXIHAL2350)

#include "grbl/hal.h"
#include "grbl/task.h"
#include "grbl/state_machine.h"
#include "grbl/pin_bits_masks.h"

//static driver_setup_ptr driver_setup;

static control_signals_get_state_ptr hal_control_get_state;
static stepper_status_t stepper_status = {};

typedef struct {
    uint8_t n_pins;
    struct {
        uint8_t axis;
        xbar_t  pin;
    } motor[N_ABC_MOTORS + 3];
} motor_pins_t;

motor_pins_t fault_signals = {};

static probe_select_ptr probe_select;
static probe_configure_ptr probe_configure;

probe_id_t active_probe = Probe_Default;

// TODO: add code guards and assign pins based on board map instead of hardcoding?
aux_ctrl_t probe_pin = { .function = Input_Probe, .port = IOPORT_UNASSIGNED, .gpio.pin = 4 };
aux_ctrl_t toolsetter_pin = { .function = Input_Toolsetter, .port = IOPORT_UNASSIGNED, .gpio.pin = 3 };

void input_add_expander_pin (xbar_t *input)
{
    if(xbar_is_motor_fault_in(input->function)){
        fault_signals.motor[fault_signals.n_pins].pin = (xbar_t)*input;
        fault_signals.motor[fault_signals.n_pins++].axis = xbar_fault_pin_to_axis(input->function);
    }
}

static void poll_motor_fault (void *data)
{
    // TODO: use interrupts rather than polling for motor faults
    stepper_status.fault.state = 0;
        
    if(settings.motor_fault_enable.mask) {

        uint_fast8_t idx;
        xbar_t *pin;

        for(idx = 0; idx < fault_signals.n_pins; idx++) {
            if(bit_istrue(settings.motor_fault_enable.mask, bit(fault_signals.motor[idx].axis))) {

                pin = &fault_signals.motor[idx].pin;

                if((pin && pin->get_value(pin) != 0.0f)^ bit_istrue(settings.motor_fault_invert.mask, bit(fault_signals.motor[idx].axis)))
                    xbar_stepper_state_set(&stepper_status.fault, fault_signals.motor[idx].axis, pin->function >= Input_MotorFaultX2);
            }
        }

        if(stepper_status.fault.state && !(state_get() & (STATE_ALARM|STATE_ESTOP))) {
            control_signals_t signals = hal_control_get_state();
            signals.motor_fault = On;
            hal.control.interrupt_callback(signals);
        }
    }

    task_add_delayed(poll_motor_fault, NULL, 25);
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

static void driverSetup (void *data)
{
    if((hal.signals_cap.motor_fault = !!settings.motor_fault_enable.value && fault_signals.n_pins)) {

        task_add_delayed(poll_motor_fault, NULL, 25);

        hal.stepper.status = getDriverStatus;

        hal_control_get_state = hal.control.get_state;
        hal.control.get_state = getControlState;
    }

    if(hal.probe.select)
        hal.probe.select(Probe_Default);
}

bool onProbeSelect (probe_id_t probe_id)
{
    switch(probe_id) {

        case Probe_Default:
            ioport_enable_irq(probe_pin.port, (pin_irq_mode_t)IRQ_Mode_All, NULL);
            ioport_enable_irq(toolsetter_pin.port, (pin_irq_mode_t)IRQ_Mode_None, NULL);
            break;    

        case Probe_Toolsetter:
            ioport_enable_irq(probe_pin.port, (pin_irq_mode_t)IRQ_Mode_None, NULL);
            ioport_enable_irq(toolsetter_pin.port, (pin_irq_mode_t)IRQ_Mode_All, NULL);
            break;

        default:
            return false;
    }

    active_probe = probe_id;
    hal.probe.configure(false, false);

    return true;
}

static void probeConfigure (bool is_probe_away, bool probing)
{
    if (active_probe == Probe_Toolsetter) {
        if (settings.probe.invert_toolsetter_input != settings.probe.invert_probe_pin)
            is_probe_away = !is_probe_away; // invert here only if toolsetter invert setting is different from probe invert setting
    }
    probe_configure(is_probe_away, probing);
}

#if PROBE_ENABLE == 2 && NGC_PARAMETERS_ENABLE

void probe_select_init (void)
{
    bool ok = true;
    xbar_t *pin;

    if(ioports_enumerate(Port_Digital, Port_Input, (pin_cap_t){.external = On, .claimable = On }, __find_in_ext, &probe_pin)){
        if(pin = ioport_claim(Port_Digital, Port_Input, &probe_pin.port, NULL)) {
            ioport_set_description(Port_Digital, Port_Input, probe_pin.port, "expander multiplex");
            ioport_set_function(pin, probe_pin.function, NULL);
        }
    } else 
        ok = false;

    if(ioports_enumerate(Port_Digital, Port_Input, (pin_cap_t){.external = On, .claimable = On }, __find_in_ext, &toolsetter_pin)){
        if(pin = ioport_claim(Port_Digital, Port_Input, &toolsetter_pin.port, NULL)) {
            ioport_set_description(Port_Digital, Port_Input, toolsetter_pin.port, "expander multiplex");
            ioport_set_function(pin, toolsetter_pin.function, NULL);
        }
    } else 
        ok = false;
    
    if(ok && (hal.driver_cap.toolsetter = On)) {
        probe_select = hal.probe.select;
        hal.probe.select = onProbeSelect;

        probe_configure = hal.probe.configure;
        hal.probe.configure = probeConfigure;

    } else
        task_run_on_startup(report_warning, "Probe expander plugin: error claiming required ports.");

}

#endif // PROBE_ENABLE == 2 && NGC_PARAMETERS_ENABLE

void board_init (void)
{
    task_run_on_startup(driverSetup, NULL);

    // driver_setup = hal.driver_setup;
    // hal.driver_setup = driverSetup;

}

#endif // BOARD_FLEXIHAL2350
