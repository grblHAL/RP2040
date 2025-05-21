/*

  sr16_out.c - driver code for PCA9654E I2C expander (output only)

  Part of grblHAL

  Copyright (c) 2018-2025 Terje Io

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

#ifdef SD_SHIFT_REGISTER

#include "hardware/pio.h"

#include "driverPIO.pio.h"

static uint16_t sr16_out = 0;
static sr_reg_t *sr16;
static io_ports_data_t digital;
static xbar_t aux_out[16] = {};
static enumerate_pins_ptr on_enumerate_pins;

static void digital_out_ll (xbar_t *output, float value)
{
    static uint16_t last_out = 0;

    bool on = value != 0.0f;

    if(aux_out[output->id].mode.inverted)
        on = !on;

    if(on)
        sr16_out |= (1 << (15 - output->pin));
    else
        sr16_out &= ~(1 << (15 - output->pin));

    if(last_out != sr16_out) {
        last_out = sr16_out;
        out_sr16_write(sr16->pio, sr16->sm, sr16_out);
    }
}

static bool digital_out_cfg (xbar_t *output, gpio_out_config_t *config, bool persistent)
{
    if(output->id < digital.out.n_ports) {

        if(config->inverted != aux_out[output->id].mode.inverted) {
            aux_out[output->id].mode.inverted = config->inverted;
            digital_out_ll(&aux_out[output->id], (float)(!(sr16_out & (1 << (15 - output->pin))) ^ config->inverted));
        }

        // Open drain not supported

        if(persistent)
            ioport_save_output_settings(output, config);
    }

    return output->id < digital.out.n_ports;
}

static void digital_out (uint8_t port, bool on)
{
    if(port < digital.out.n_ports)
        digital_out_ll(&aux_out[port], (float)on);
}

static float digital_out_state (xbar_t *output)
{
    float value = -1.0f;

    if(output->id < digital.out.n_ports)
        value = (float)((sr16_out & (1 << (15 - output->pin))) != 0);

    return value;
}

static bool set_function (xbar_t *output, pin_function_t function)
{
    if(output->id < digital.out.n_ports)
        aux_out[output->id].function = function;

    return output->id < digital.out.n_ports;
}

static xbar_t *get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    if(dir == Port_Output && port < digital.out.n_ports) {
        memcpy(&pin, &aux_out[port], sizeof(xbar_t));
        pin.ports_id = &digital;
        pin.get_value = digital_out_state;
        pin.set_value = digital_out_ll;
        pin.set_function = set_function;
        pin.config = digital_out_cfg;
        info = &pin;
    }

    return info;
}

static void set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Output && port < digital.out.n_ports)
        aux_out[port].description = description;
}

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    on_enumerate_pins(low_level, pin_info, data);

    uint_fast8_t idx;

    for(idx = 0; idx < digital.out.n_ports; idx++) {

        memcpy(&pin, &aux_out[idx], sizeof(xbar_t));

        if(!low_level)
            pin.port = "SR16.";

        pin_info(&pin, data);
    };
}

static void get_aux_max (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxOutput)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

void sr16_init (sr_reg_t *reg)
{
    uint_fast8_t idx;
    pin_function_t aux_out_base = Output_Aux0;

    io_digital_t dports = {
        .ports = &digital,
        .digital_out = digital_out,
        .get_pin_info = get_pin_info,
        .set_pin_description = set_pin_description,
    };

    hal.enumerate_pins(false, get_aux_max, &aux_out_base);

    digital.out.n_ports = 16;

    for(idx = 0; idx < digital.out.n_ports; idx++) {
        uint8_t id = (idx > 7 ? idx - 8 : idx + 8);
        aux_out[id].pin = id;
        aux_out[id].port = &sr16_out;
        aux_out[id].id = id;
        aux_out[id].function = aux_out_base + id;
        aux_out[id].group = PinGroup_AuxOutput;
        aux_out[id].cap.output = On;
        aux_out[id].cap.external = On;
        aux_out[id].cap.claimable = On;
        aux_out[id].mode.output = On;
    }

    // TODO: claim PIO state machine here
    if(ioports_add_digital(&dports))
        sr16 = reg;

    on_enumerate_pins = hal.enumerate_pins;
    hal.enumerate_pins = onEnumeratePins;
}

#endif // SD_SHIFT_REGISTER
