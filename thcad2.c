/*
  thcad2.c - analog input from frequency, for Mesa THCAD2 converter

  Part of grblHAL

  Copyright (c) 2023 Jeremy P Bentham and 2025 Terje Io

  Idea/code adapted from https://iosoft.blog/2023/07/30/picofreq/

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

#if THCAD2_ENABLE

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"

#include "grbl/plugins.h"
#include "grbl/ioports.h"

#define THCAD2_PIN      11
#define GATE_TIMER_PIN  0

static float value = -1.0f;
static enumerate_pins_ptr on_enumerate_pins;
static io_ports_data_t analog = {};

uint counter_slice, gate_slice;
uint timer_dma_chan;
uint gate_dma_chan, gate_dma_dreq, csr_stopval;
uint32_t timers_enable;

static xbar_t thcad2 = {
    .id = 0,
    .pin = THCAD2_PIN,
    .function = Input_Analog_Aux0,
    .group = PinGroup_AuxInputAnalog,
    .port = &value,
    .cap = {
        .input = On,
        .analog = On,
        .resolution = Resolution_12bit,
        .external = On,
        .claimable = On
    },
    .mode = {
        .output = Off,
        .analog = On
    }
};

static float thcad2_in_state (xbar_t *input)
{
    return value;
}

static int32_t thcad2_wait_on_input (uint8_t port, wait_mode_t wait_mode, float timeout)
{
    return port < analog.in.n_ports ? (int32_t)value : -1;
}

static bool set_pin_function (xbar_t *input, pin_function_t function)
{
    if(input->id == thcad2.id)
        thcad2.function = function;

    return input->id == thcad2.id;
}

static xbar_t *thcad2_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    memcpy(&pin, &thcad2, sizeof(xbar_t));

    if(dir == Port_Input && port < analog.in.n_ports) {
        pin.get_value = thcad2_in_state;
        pin.set_function = set_pin_function;
        info = &pin;
    }

    return info;
}

static void thcad2_set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Input && port < analog.in.n_ports)
        thcad2.description = description;
}

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    on_enumerate_pins(low_level, pin_info, data);

    memcpy(&pin, &thcad2, sizeof(xbar_t));

    if(!low_level)
        pin.port = "thcad2:";

    pin_info(&pin, data);
}

static void get_next_port (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxInputAnalog)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

void get_sample (void)
{
    dma_hw->ints0 = 1u << gate_dma_chan;

    pwm_set_enabled(gate_slice, false);
    value = (float)pwm_get_counter(counter_slice);

    dma_channel_transfer_from_buffer_now(gate_dma_chan, &csr_stopval, 1);
    pwm_set_counter(counter_slice, 0);
    pwm_set_counter(gate_slice, 0);
    pwm_set_mask_enabled(timers_enable);
}

void thcad2_init (void)
{
    io_analog_t ports = {
        .ports = &analog,
        .get_pin_info = thcad2_get_pin_info,
        .wait_on_input = thcad2_wait_on_input,
        .set_pin_description = thcad2_set_pin_description
    };

    hal.enumerate_pins(false, get_next_port, &thcad2.function);

    analog.in.n_ports = 1;
    
    if(ioports_add_analog(&ports)) {

        uint32_t clk = clock_get_hz(clk_sys) / 1000000;

        counter_slice = pwm_gpio_to_slice_num(thcad2.pin);

        gpio_set_function(THCAD2_PIN, GPIO_FUNC_PWM);
        pwm_config pwm_cfg = pwm_get_default_config();
        pwm_config_set_clkdiv_mode(&pwm_cfg, PWM_DIV_B_RISING);
        pwm_config_set_clkdiv(&pwm_cfg, 1);
        pwm_init(counter_slice, &pwm_cfg, false);
        
        gate_slice = pwm_gpio_to_slice_num(GATE_TIMER_PIN);
        io_rw_32 *counter_slice_csr = &pwm_hw->slice[counter_slice].csr;

        pwm_set_clkdiv_int_frac4(gate_slice, clk, 0);
        pwm_set_wrap(gate_slice, 10000); // 10ms
        pwm_set_phase_correct(gate_slice, false);

        timers_enable = (1 << counter_slice) | (1 << gate_slice);

        gate_dma_chan = dma_claim_unused_channel(true);
        dma_channel_config cfg = dma_channel_get_default_config(gate_dma_chan);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
        channel_config_set_read_increment(&cfg, false);
        channel_config_set_dreq(&cfg, pwm_get_dreq(gate_slice));
        csr_stopval = *counter_slice_csr;
        dma_channel_configure(gate_dma_chan, &cfg, counter_slice_csr, &csr_stopval, 1, false);
        pwm_set_enabled(gate_slice, true);

        dma_channel_set_irq0_enabled(gate_dma_chan, true);

        irq_set_exclusive_handler(DMA_IRQ_0, get_sample);
        irq_set_enabled(DMA_IRQ_0, true);

        get_sample();
    }

    on_enumerate_pins = hal.enumerate_pins;
    hal.enumerate_pins = onEnumeratePins;
}

#endif // THCAD2_ENABLE
