/*
  ioports_analog.c - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2023-2025 Terje Io

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

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "grbl/ioports.h"

static io_ports_data_t analog;
static input_signal_t *aux_in_analog;
static output_signal_t *aux_out_analog;
static ioports_pwm_t *pwm_data;
static float *pwm_values;

static void set_pwm_cap (xbar_t *output, bool servo_pwm)
{
    if(output && output->id < analog.out.n_ports) {
        aux_out_analog[output->id].mode.pwm = !servo_pwm;
        aux_out_analog[output->id].mode.servo_pwm = servo_pwm;
    }
}

static bool init_pwm (xbar_t *output, pwm_config_t *config, bool persistent)
{
    bool ok;
    ioports_pwm_t *pwm_data = (ioports_pwm_t *)output->port;
    uint32_t prescaler = config->freq_hz > 2000.0f ? 1 : (config->freq_hz > 200.0f ? 12 : 50);

    if((ok = ioports_precompute_pwm_values(config, pwm_data, clock_get_hz(clk_sys) / prescaler))) {

        pwm_config pwm_config = pwm_get_default_config();
        pwm_config_set_clkdiv_int(&pwm_config, prescaler);
        pwm_config_set_wrap(&pwm_config, pwm_data->period);

        gpio_init(output->pin);
        gpio_set_dir_out_masked(1 << output->pin);
        gpio_set_function(output->pin, GPIO_FUNC_PWM);
        pwm_set_gpio_level(output->pin, pwm_data->off_value);

        uint channel = pwm_gpio_to_channel(output->pin);
        pwm_config_set_output_polarity(&pwm_config, (!channel & config->invert), (channel & config->invert));

        pwm_init(pwm_gpio_to_slice_num(output->pin), &pwm_config, true);

        set_pwm_cap(output, config->servo_mode);
    }
    
    return ok;
}

static float pwm_get_value (xbar_t *output)
{
    return pwm_values && output->id < analog.out.n_ports ? pwm_values[output->id] : -1.0f;
}

static bool analog_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports) {
        if(pwm_values)
            pwm_values[aux_out_analog[port].id - Output_Analog_Aux0] = value;
        pwm_set_gpio_level(aux_out_analog[port].pin, ioports_compute_pwm_value(&pwm_data[aux_out_analog[port].pwm_idx], value));
    }

    return port < analog.out.n_ports;
}

static bool set_function (xbar_t *port, pin_function_t function)
{
    if(port->mode.output)
        aux_in_analog[port->id].id = function;

    return port->mode.output;
}

static xbar_t *get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;
 
    xbar_t *info = NULL;

    memset(&pin, 0, sizeof(xbar_t));

    switch(dir) {

        case Port_Output: 
            if(port < analog.out.n_ports) {
                pin.id = port;
                pin.mode = aux_out_analog[pin.id].mode;
                pin.mode.pwm = !pin.mode.servo_pwm; //?? for easy filtering
                XBAR_SET_CAP(pin.cap, pin.mode);
                pin.function = aux_out_analog[pin.id].id;
                pin.group = aux_out_analog[pin.id].group;
                pin.pin = aux_out_analog[pin.id].pin;
                pin.description = aux_out_analog[pin.id].description;
                pin.set_function = set_function;
                if(aux_out_analog[pin.id].mode.pwm || aux_out_analog[pin.id].mode.servo_pwm) {
                    pin.port = &pwm_data[aux_out_analog[pin.id].pwm_idx];
                    pin.config = (xbar_config_ptr)init_pwm;
                    pin.get_value = pwm_get_value;
                }
                info = &pin;
            }
            break;
    }

    return info;
}

static void set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Input && port < analog.in.n_ports)
        aux_in_analog[port].description = description;
    else if(port < analog.out.n_ports)
        aux_out_analog[port].description = description;
}

void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs)
{
    io_analog_t ports = {
        .ports = &analog,
        .analog_out = analog_out,
        .get_pin_info = get_pin_info,
//        .wait_on_input = wait_on_input,
        .set_pin_description = set_pin_description
    };

    aux_in_analog = aux_inputs->pins.inputs;
    aux_out_analog = aux_outputs->pins.outputs;

    analog.in.n_ports = aux_inputs->n_pins;
    analog.out.n_ports = aux_outputs->n_pins;

    if(ioports_add_analog(&ports)) {

        uint_fast8_t i, n_pwm = 0;

        if(analog.out.n_ports) {

            pwm_config_t config = {
                .freq_hz = 5000.0f,
                .min = 0.0f,
                .max = 100.0f,
                .off_value = 0.0f,
                .min_value = 0.0f,
                .max_value = 100.0f,
                .invert = Off
            };

            hal.port.analog_out = analog_out;

            for(i = 0; i < analog.out.n_ports; i++) {
                if(aux_out_analog[i].mode.pwm)
                    n_pwm++;
            }

            pwm_data = calloc(n_pwm, sizeof(ioports_pwm_t));
            pwm_values = calloc(n_pwm, sizeof(float));

            n_pwm = 0;
            for(i = 0; i < analog.out.n_ports; i++) {
                if(aux_out_analog[i].mode.pwm && !!pwm_data) {
                    aux_out_analog[i].pwm_idx = n_pwm++;
                    init_pwm(get_pin_info(Port_Output, i), &config, false);
                }
            }
        }
    }
}
