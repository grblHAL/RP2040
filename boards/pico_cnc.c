/*
  pico_cnc.c - driver code for RP2040 ARM processors

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

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver.h"

#if defined(BOARD_PICO_CNC)

#include "hardware/pio.h"

#include "driverPIO.pio.h"
#include "grbl/protocol.h"

static sr_reg_t *sr16;
static bool state[AUX_N_OUT];

#if NUM_BANK0_GPIOS <= 32
static volatile uint32_t event_bits;
#endif
static io_ports_data_t digital;
static input_signal_t *aux_in;
static output_signal_t *aux_out;

static bool digital_out_cfg (xbar_t *output, gpio_out_config_t *config, bool persistent)
{
    if(output->id < digital.out.n_ports) {

        if(config->inverted != aux_out[output->id].mode.inverted) {
            aux_out[output->id].mode.inverted = config->inverted;
            state[output->id] = !state[output->id];
            out_sr16_write(sr16->pio, sr16->sm, sr16->reg->value);
        }

        // Open drain not supported

        if(persistent)
            ioport_save_output_settings(output, config);
    }

    return aux_out->id < digital.out.n_ports;
}

static void digital_out (uint8_t port, bool on)
{
    if(sr16 && port < digital.out.n_ports) {

        on = ((settings.ioport.invert_out.mask >> port) & 0x01) ? !on : on;

        state[port] = on;

        switch(port)
        {
            case 0:
                sr16->reg->aux0_out = on;
                break;

            case 1:
                sr16->reg->aux1_out = on;
                break;	

            case 2:
                sr16->reg->aux2_out = on;
                break;	

            case 3:
                sr16->reg->aux3_out = on;
                break;	

            case 4:
                sr16->reg->aux4_out = on;
                break;	

            case 5:
                sr16->reg->aux5_out = on;
                break;	

            case 6:
                sr16->reg->aux6_out = on;
                break;

#ifdef AUXOUTPUT7_PIN // TX enable
            case 7:
                DIGITAL_OUT(AUXOUTPUT7_PIN, on);
                break;
#endif

            default:
                break;
        }

        out_sr16_write(sr16->pio, sr16->sm, sr16->reg->value);
    }
}

static float digital_out_state (xbar_t *output)
{
    float value = -1.0f;

    if(output->id < digital.out.n_ports)
        value = (float)state[output->id];

    return value;
}

static bool digital_in_cfg (xbar_t *input, gpio_in_config_t *config, bool persistent)
{
    if(input->id < digital.in.n_ports && config->pull_mode != PullMode_UpDown) {

        aux_in[input->id].mode.inverted = config->inverted;
        aux_in[input->id].mode.debounce = config->debounce;
        aux_in[input->id].mode.pull_mode = config->pull_mode;
        gpio_set_inover(input->pin, config->inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
        gpio_set_pulls(input->pin, config->pull_mode == PullMode_Up, config->pull_mode == PullMode_Down);

        if(persistent)
            ioport_save_input_settings(input, config);
    }

    return input->id < digital.in.n_ports;
}

static float digital_in_state (xbar_t *input)
{
    float value = -1.0f;

    if(input->id < digital.in.n_ports)
        value = (float)(DIGITAL_IN(aux_in[input->id].pin));

    return value;
}

inline static __attribute__((always_inline)) int32_t get_input (const input_signal_t *input, wait_mode_t wait_mode, float timeout)
{
    if(wait_mode == WaitMode_Immediate)
        return DIGITAL_IN(input->pin);

    int32_t value = -1;
    uint_fast16_t delay = (uint_fast16_t)ceilf((1000.0f / 50.0f) * timeout) + 1;

    if(wait_mode == WaitMode_Rise || wait_mode == WaitMode_Fall) {

        pin_irq_mode_t irq_mode = wait_mode == WaitMode_Rise ? IRQ_Mode_Rising : IRQ_Mode_Falling;

        if(input->cap.irq_mode & irq_mode) {

            event_bits &= ~(1UL << input->pin);
            pinEnableIRQ(input, irq_mode);

            do {
                if(event_bits & (1UL << input->pin)) {
                    value = DIGITAL_IN(input->pin);
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
            if((DIGITAL_IN(input->pin)) == wait_for) {
                value = DIGITAL_IN(input->pin);
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

void ioports_event (input_signal_t *input)
{
    event_bits |= (1UL << input->pin);

    if(input->interrupt_callback)
        input->interrupt_callback(input->user_port, DIGITAL_IN(input->pin) ^ input->mode.inverted);
}

static int32_t wait_on_input (io_port_type_t type, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    if(port < digital.in.n_ports)
        value = get_input(&aux_in[port], wait_mode, timeout);

    return value;
}

static bool register_interrupt_handler (uint8_t port, uint8_t user_port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr interrupt_callback)
{
    bool ok;

    if((ok = port < digital.in.n_ports && aux_in[port].cap.irq_mode != IRQ_Mode_None)) {

        input_signal_t *input = &aux_in[port];

        if(irq_mode != IRQ_Mode_None && (ok = interrupt_callback != NULL)) {
            input->user_port = user_port;
            input->mode.irq_mode = irq_mode;
            input->interrupt_callback = interrupt_callback;
            pinEnableIRQ(input, irq_mode);
        }

        if(irq_mode == IRQ_Mode_None || !ok) {
            hal.irq_disable();
            pinEnableIRQ(input, IRQ_Mode_None);
            input->mode.irq_mode = IRQ_Mode_None;
            input->interrupt_callback = NULL;
            hal.irq_enable();
        }
    }

    return ok;
}

static xbar_t *get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    if(dir == Port_Input && port < digital.in.n_ports) {
        XBAR_SET_DIN_INFO(pin, port, aux_in[pin.id], digital_in_cfg, digital_in_state);
        info = &pin;
    }

    if(dir == Port_Output && port < digital.out.n_ports) {
        XBAR_SET_DOUT_INFO(pin, port, aux_out[pin.id], digital_out_cfg, digital_out_state);
        info = &pin;
    }

    return info;
}

static void set_pin_description (io_port_direction_t dir, uint8_t port, const char *s)
{
    if(dir == Port_Input && port < digital.in.n_ports)
        aux_in[port].description = s;

    if(dir == Port_Output && port < digital.out.n_ports)
        aux_out[port].description = s;
}

void board_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs, sr_reg_t *reg)
{
    aux_in = aux_inputs->pins.inputs;
    aux_out = aux_outputs->pins.outputs;

    digital.in.n_ports = aux_inputs->n_pins;
    digital.out.n_ports = aux_outputs->n_pins;

    io_digital_t ports = {
        .ports = &digital,
        .digital_out = digital_out,
        .get_pin_info = get_pin_info,
 //       .wait_on_input = wait_on_input,
        .set_pin_description = set_pin_description,
        .register_interrupt_handler = register_interrupt_handler
    };

    if(ioports_add_digital(&ports))
        sr16 = reg;
}

#endif // defined(BOARD_PICO_CNC)
