/*
  ioports_analog.c - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2023 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
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

static wait_on_input_ptr wait_on_input_digital;
static set_pin_description_ptr set_pin_description_digital;
static get_pin_info_ptr get_pin_info_digital;
static claim_port_ptr claim_digital;
static swap_pins_ptr swap_pins_digital; 

static bool init_pwm (xbar_t *output, pwm_config_t *config)
{
    bool ok;
    ioports_pwm_t *pwm_data = (ioports_pwm_t *)output->port;
    uint32_t prescaler = config->freq_hz > 2000.0f ? 1 : (config->freq_hz > 200.0f ? 12 : 50);

    if((ok = ioports_precompute_pwm_values(config, pwm_data, clock_get_hz(clk_sys) / prescaler))) {

        pwm_config pwm_config = pwm_get_default_config();
        pwm_config_set_clkdiv_int(&pwm_config, prescaler);
        pwm_config_set_wrap(&pwm_config, pwm_data->period);

        gpio_set_function(output->pin, GPIO_FUNC_PWM);
        pwm_set_gpio_level(output->pin, pwm_data->off_value);

        uint channel = pwm_gpio_to_channel(output->pin);
        pwm_config_set_output_polarity(&pwm_config, (!channel & config->invert), (channel & config->invert));

        pwm_init(pwm_gpio_to_slice_num(output->pin), &pwm_config, true);
    }
    
    return ok;
}

static bool analog_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports) {
        port = ioports_map(analog.out, port);
        pwm_set_gpio_level(aux_out_analog[port].pin, ioports_compute_pwm_value(&pwm_data[aux_out_analog[port].pwm_idx], value));
    }

    return port < analog.out.n_ports;
}

static xbar_t *get_pin_info (io_port_type_t type, io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;
    xbar_t *info = NULL;

    memset(&pin, 0, sizeof(xbar_t));

    if(type == Port_Digital)
        return get_pin_info_digital ? get_pin_info_digital(type, dir, port) : NULL;

    else if(dir == Port_Output) {

        if(dir == Port_Output && port < analog.out.n_ports) {
            port = ioports_map(analog.out, port);
            pin.mode = aux_out_analog[port].mode;
            pin.mode.output = pin.mode.analog = On;
            pin.cap = pin.mode;
            pin.function = aux_out_analog[port].id;
            pin.group = aux_out_analog[port].group;
            pin.pin = aux_out_analog[port].pin;
            pin.bit = 1 << aux_out_analog[port].pin;
            pin.description = aux_out_analog[port].description;
            if(aux_out_analog[port].mode.pwm) {
                pin.port = &pwm_data[aux_out_analog[port].pwm_idx];
                pin.config = (xbar_config_ptr)init_pwm;
            }
            info = &pin;
        }
    }

    return info;
}

static void set_pin_description (io_port_type_t type, io_port_direction_t dir, uint8_t port, const char *description)
{
    if(type == Port_Analog) {
        if(dir == Port_Output && port < analog.out.n_ports)
            aux_out_analog[ioports_map(analog.out, port)].description = description;
    } else if(set_pin_description_digital)
        set_pin_description_digital(type, dir, port, description);
}

static bool claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description)
{
    bool ok = false;

    if(type == Port_Digital)
        return claim_digital ? claim_digital(type, dir, port, description) : false;

    else if(dir == Port_Output) {

        if((ok = analog.out.map && *port < analog.out.n_ports && !aux_out_analog[*port].mode.claimed)) {

            uint8_t i;

            hal.port.num_analog_out--;

            for(i = ioports_map_reverse(&analog.out, *port); i < hal.port.num_analog_out; i++) {
                analog.out.map[i] = analog.out.map[i + 1];
                aux_out_analog[analog.out.map[i]].description = iports_get_pnum(analog, i);
            }

            aux_out_analog[*port].mode.claimed = On;
            aux_out_analog[*port].description = description;

            analog.out.map[hal.port.num_analog_out] = *port;
            *port = hal.port.num_analog_out;
        }
    }

    return ok;
}

void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs)
{
    aux_in_analog = aux_inputs->pins.inputs;
    aux_out_analog = aux_outputs->pins.outputs;

    set_pin_description_digital = hal.port.set_pin_description;
    hal.port.set_pin_description = set_pin_description;

    if(ioports_add(&analog, Port_Analog, aux_inputs->n_pins, aux_outputs->n_pins))  {

        uint_fast8_t i, n_pwm = 0;
/*
        if(analog.n_in) {
            wait_on_input_digital = hal.port.wait_on_input;
            hal.port.wait_on_input = wait_on_input;
        }
*/
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

            n_pwm = 0;
            for(i = 0; i < analog.out.n_ports; i++) {
                if(aux_out_analog[i].mode.pwm && !!pwm_data) {
                    aux_out_analog[i].pwm_idx = n_pwm++;
                    init_pwm(get_pin_info(Port_Analog, Port_Output, i), &config);
                }
            }
        }

        claim_digital = hal.port.claim;
        swap_pins_digital = hal.port.swap_pins;
        get_pin_info_digital = hal.port.get_pin_info;
 
        hal.port.claim = claim;
//        hal.port.swap_pins = swap_pins;
        hal.port.get_pin_info = get_pin_info;
    }
}
