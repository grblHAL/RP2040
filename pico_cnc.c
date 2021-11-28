/*
  pico_cnc.c - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021 Terje Io
  
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

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver.h"

#if defined(BOARD_PICO_CNC)

#include "hardware/pio.h"

#include "driverPIO.pio.h"
#include "grbl/protocol.h"

static output_sr_t *sr;
static bool state[AUX_N_OUT];

static char *pnum = NULL;
static uint8_t n_in, n_out, *in_map = NULL, *out_map = NULL;
static volatile uint32_t event_bits;
static volatile bool spin_lock = false;
static input_signal_t *aux_in;
static output_signal_t *aux_out;
static char input_ports[50] = "", output_ports[50] = "";

static void aux_settings_load (void);
static bool is_setting_available (const setting_detail_t *setting);
static status_code_t aux_set_invert_out (setting_id_t id, uint_fast16_t int_value);
static uint32_t aux_get_invert_out (setting_id_t setting);
static void digital_out (uint8_t port, bool on);

static const setting_group_detail_t aux_groups[] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t aux_settings[] = {
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, output_ports, NULL, NULL, Setting_NonCoreFn, aux_set_invert_out, aux_get_invert_out },
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, input_ports, NULL, NULL, Setting_NonCore, &settings.ioport.invert_in.mask, NULL, is_setting_available },
};

static setting_details_t setting_details = {
    .groups = aux_groups,
    .n_groups = sizeof(aux_groups) / sizeof(setting_group_detail_t),
    .settings = aux_settings,
    .n_settings = sizeof(aux_settings) / sizeof(setting_detail_t),
    .load = aux_settings_load,
    .save = settings_write_global
};

static bool is_setting_available (const setting_detail_t *setting)
{
    bool available = false;

    switch(setting->id) {

        case Settings_IoPort_InvertIn:
//        case Settings_IoPort_Pullup_Disable:
            available = n_in > 0;
            break;

        default:
            break;
    }

    return available;
}

static void aux_settings_load (void)
{
    uint_fast8_t idx = AUX_N_OUT;

    do {
        idx--;
		digital_out(idx, false);
    } while(idx);
}

static status_code_t aux_set_invert_out (setting_id_t id, uint_fast16_t value)
{
    ioport_bus_t invert;
    invert.mask = (uint8_t)value & AUX_OUT_MASK;

    if(invert.mask != settings.ioport.invert_out.mask) {
        uint_fast8_t idx = AUX_N_OUT;
        do {
            idx--;
            if(((settings.ioport.invert_out.mask >> idx) & 0x01) != ((invert.mask >> idx) & 0x01))
                digital_out(idx, !state[idx]);
        } while(idx);

        settings.ioport.invert_out.mask = invert.mask;
    }

    return Status_OK;
}

static uint32_t aux_get_invert_out (setting_id_t setting)
{
    return settings.ioport.invert_out.mask;
}

static void digital_out (uint8_t port, bool on)
{
    if(port < n_out) {

        if(out_map)
            port = out_map[port];

        on = ((settings.ioport.invert_out.mask >> port) & 0x01) ? !on : on;

        state[port] = on;

        switch(port)
        {
            case 0:
                sr->aux0_out = on;
                break;

            case 1:
                sr->aux1_out = on;
                break;	

            case 2:
                sr->aux2_out = on;
                break;	

            case 3:
                sr->aux3_out = on;
                break;	

            case 4:
                sr->aux4_out = on;
                break;	

            case 5:
                sr->aux5_out = on;
                break;	

            case 6:
                sr->aux6_out = on;
                break;

            case 7:
                sr->aux7_out = on;
                break;
                
            default:
                break;
        }

        out_sr16_write(pio1, 1, sr->value);
    }
}

inline static __attribute__((always_inline)) int32_t get_input (const input_signal_t *input, bool invert, wait_mode_t wait_mode, float timeout)
{
    if(wait_mode == WaitMode_Immediate)
        return DIGITAL_IN(input->bit) ^ invert;

    int32_t value = -1;
    uint_fast16_t delay = (uint_fast16_t)ceilf((1000.0f / 50.0f) * timeout) + 1;

    if(wait_mode == WaitMode_Rise || wait_mode == WaitMode_Fall) {

        pin_irq_mode_t irq_mode = wait_mode == WaitMode_Rise ? IRQ_Mode_Rising : IRQ_Mode_Falling;

        if(input->cap.irq_mode & irq_mode) {

            event_bits &= ~input->bit;
            pinEnableIRQ(input, irq_mode);

            do {
                if(event_bits & input->bit) {
                    value = DIGITAL_IN(input->bit) ^ invert;
                    break;
                }
                if(delay) {
                    protocol_execute_realtime();
                    hal.delay_ms(50, NULL);
                } else
                    break;
            } while(--delay && !sys.abort);

            pinEnableIRQ(input, IRQ_Mode_None);    // Restore pin interrupt status
        }

    } else {

        bool wait_for = wait_mode != WaitMode_Low;

        do {
            if((DIGITAL_IN(input->bit) ^ invert) == wait_for) {
                value = DIGITAL_IN(input->bit);
                break;
            }
            if(delay) {
                protocol_execute_realtime();
                hal.delay_ms(50, NULL);
            } else
                break;
        } while(--delay && !sys.abort);
    }

    return value;
}

inline static __attribute__((always_inline)) uint8_t in_map_rev (uint8_t port)
{
    if(in_map) {
        uint_fast8_t idx = n_in;
        do {
            if(in_map[--idx] == port) {
                port = idx;
                break;
            }
        } while(idx);
    }

    return port;
}

inline static __attribute__((always_inline)) uint8_t out_map_rev (uint8_t port)
{
    if(out_map) {
        uint_fast8_t idx = n_out;
        do {
            if(out_map[--idx] == port) {
                port = idx;
                break;
            }
        } while(idx);
    }

    return port;
}

void ioports_event (input_signal_t *input)
{
    spin_lock = true;
    event_bits |= input->bit;

    if(input->interrupt_callback)
        input->interrupt_callback(in_map_rev(input->id - Input_Aux0), DIGITAL_IN(input->bit));

    spin_lock = false;
}

static int32_t wait_on_input (io_port_type_t type, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    if(type == Port_Digital && port < n_in) {
        if(in_map)
            port = in_map[port];
        value = get_input(&aux_in[port], (settings.ioport.invert_in.mask << port) & 0x01, wait_mode, timeout);
    }
//    else if(port == 0)
//        value = analogRead(41);

    return value;
}

static bool register_interrupt_handler (uint8_t port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr interrupt_callback)
{
    bool ok;

    if(in_map)
        port = in_map[port];

    if((ok = port < n_in && aux_in[port].cap.irq_mode != IRQ_Mode_None)) {

        input_signal_t *input = &aux_in[port];

        if(irq_mode != IRQ_Mode_None && (ok = interrupt_callback != NULL)) {
            input->irq_mode = irq_mode;
            input->interrupt_callback = interrupt_callback;
            pinEnableIRQ(input, irq_mode);
        }

        if(irq_mode == IRQ_Mode_None || !ok) {
            while(spin_lock);
            pinEnableIRQ(input, IRQ_Mode_None);
            input->irq_mode = IRQ_Mode_None;
            input->interrupt_callback = NULL;
        }
    }

    return ok;
}

static xbar_t *get_pin_info (io_port_type_t type, io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;
    xbar_t *info = NULL;

    if(type == Port_Digital) {

        memset(&pin, 0, sizeof(xbar_t));

        if(dir == Port_Input && port < n_in) {
            if(in_map)
                port = in_map[port];
            pin.mode.input = On;
            pin.mode.irq_mode = aux_in[port].irq_mode;
            pin.mode.can_remap = !aux_in[port].cap.remapped;
            pin.cap = aux_in[port].cap;
            pin.function = aux_in[port].id;
            pin.group = aux_in[port].group;
            pin.pin = aux_in[port].pin;
            pin.bit = aux_in[port].bit;
            pin.description = aux_in[port].description;
            info = &pin;
        }

        if(dir == Port_Output && port < n_out) {
            if(out_map)
                port = out_map[port];
            pin.mode = aux_out[port].mode;
            pin.mode.output = On;
            pin.function = aux_out[port].id;
            pin.group = aux_out[port].group;
            pin.pin = aux_out[port].pin;
            pin.bit = 1 << aux_out[port].pin;
 //           pin.port = (void *)aux_out[port].port;
            pin.description = aux_out[port].description;
            info = &pin;
        }
    }

    return info;
}

static void set_pin_description (io_port_type_t type, io_port_direction_t dir, uint8_t port, const char *s)
{
    if(type == Port_Digital) {
        if(dir == Port_Input && port < n_in)
            aux_in[in_map ? in_map[port] : port].description = s;

        if(dir == Port_Output && port < n_out)
            aux_out[out_map ? out_map[port] : port].description = s;
    }
}

static char *get_pnum (uint8_t port)
{
    return pnum ? (pnum + (port * 3) + (port > 9 ? port - 10 : 0)) : NULL;
}

static bool claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description)
{
    bool ok = false;

    if(type == Port_Digital) {

        if(dir == Port_Input) {

            if((ok = in_map && *port < n_in && !aux_in[*port].cap.claimed)) {

                uint8_t i;

                hal.port.num_digital_in--;

                for(i = in_map_rev(*port); i < hal.port.num_digital_in ; i++) {
                    in_map[i] = in_map[i + 1];
                    aux_in[in_map[i]].description = get_pnum(i);
                }

                aux_in[*port].cap.claimed = On;
                aux_in[*port].description = description;

                in_map[hal.port.num_digital_in] = *port;
                *port = hal.port.num_digital_in;
            }

        } else if((ok = out_map && *port < n_out && !aux_out[*port].mode.claimed)) {

            uint8_t i;

            hal.port.num_digital_out--;

            for(i = out_map_rev(*port); i < hal.port.num_digital_out; i++) {
                out_map[i] = out_map[i + 1];
                aux_out[out_map[i]].description = get_pnum(i);
            }

            aux_out[*port].mode.claimed = On;
            aux_out[*port].description = description;

            out_map[hal.port.num_digital_out] = *port;
            *port = hal.port.num_digital_out;
        }
    }

    return ok;
}

bool swap_pins (io_port_type_t type, io_port_direction_t dir, uint8_t port_a, uint8_t port_b)
{
    bool ok = port_a == port_b;

    if(!ok && type == Port_Digital) {

        if((ok = dir == Port_Input && port_a < n_in && port_b < n_in &&
                   aux_in[port_a].interrupt_callback == NULL &&
                    aux_in[port_b].interrupt_callback == NULL)) {

            input_signal_t tmp;

            memcpy(&tmp, &aux_in[port_a], sizeof(input_signal_t));
            memcpy(&aux_in[port_a], &aux_in[port_b], sizeof(input_signal_t));
            aux_in[port_a].description = tmp.description;
            tmp.description = aux_in[port_b].description;
            memcpy(&aux_in[port_b], &tmp, sizeof(input_signal_t));
        }

        if((ok = dir == Port_Output && port_a < n_out && port_b < n_out)) {

            output_signal_t tmp;

            memcpy(&tmp, &aux_out[port_a], sizeof(output_signal_t));
            memcpy(&aux_out[port_a], &aux_out[port_b], sizeof(output_signal_t));
            aux_out[port_a].description = tmp.description;
            tmp.description = aux_out[port_b].description;
            memcpy(&aux_out[port_b], &tmp, sizeof(output_signal_t));
        }
    }

    return ok;
}

void board_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs, output_sr_t *reg)
{
    uint_fast8_t i, ports;

    sr = reg;
    aux_in = aux_inputs->pins.inputs;
    aux_out = aux_outputs->pins.outputs;

    if((hal.port.num_digital_in = n_in = aux_inputs->n_pins)) {
        hal.port.wait_on_input = wait_on_input;
        hal.port.register_interrupt_handler = register_interrupt_handler;
        in_map = malloc(n_in * sizeof(uint8_t));
    }

    if((hal.port.num_digital_out = n_out = aux_outputs->n_pins)) {
        hal.port.digital_out = digital_out;
        out_map = malloc(n_out * sizeof(uint8_t));
    }

    if((ports = max(n_in, n_out)) > 0)  {

        char *pn;

        hal.port.claim = claim;
        hal.port.swap_pins = swap_pins;
        hal.port.get_pin_info = get_pin_info;
        hal.port.set_pin_description = set_pin_description;

        settings_register(&setting_details);

        // Add M62-M65 port number mappings (P<n>) to description
        pnum = pn = malloc((3 * ports + (ports > 9 ? ports - 10 : 0)) + 1);

        for(i = 0; i < ports; i++) {

            if(pn) {
                *pn = 'P';
                strcpy(pn + 1, uitoa(i));
            }

            if(hal.port.num_digital_in && i < hal.port.num_digital_in) {
                if(in_map)
                    in_map[i] = i;
                if(pn)
                    aux_in[i].description = pn;
            }

            if(hal.port.num_digital_out && i < hal.port.num_digital_out) {
                if(out_map)
                    out_map[i] = i;
                if(pn)
                    aux_out[i].description = pn;
            }

            if(pn)
                pn += i > 9 ? 4 : 3;
        }

        // Add port names for ports up to 8 for $-setting flags
        for(i = 0; i < min(hal.port.num_digital_in, 8); i++) {
            strcat(input_ports, i == 0 ? "Aux " : ",Aux ");
            strcat(input_ports, uitoa(i));
        }

        for(i = 0; i < min(hal.port.num_digital_out, 8) ; i++) {
         //   out.mask = (out.mask << 1) + 1;
            strcat(output_ports, i == 0 ? "Aux " : ",Aux ");
            strcat(output_ports, uitoa(i));
        }
    }
}

#endif
