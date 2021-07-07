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
#include <string.h>

#include "driver.h"

#if defined(BOARD_PICO_CNC)

#include "hardware/pio.h"

#include "driverPIO.pio.h"
#include "grbl/protocol.h"

static output_sr_t *sr;
static bool state[AUX_N_OUT];

static uint_fast8_t n_in, n_out;
static volatile uint32_t event_bits;
static volatile bool spin_lock = false;
static input_signal_t *aux_in;
static char input_ports[56] = "";

static void aux_settings_load (void);
static bool is_setting_available (const setting_detail_t *setting);
static status_code_t aux_set_invert_out (setting_id_t id, uint_fast16_t int_value);
static uint32_t aux_get_invert_out (setting_id_t setting);
static void digital_out (uint8_t port, bool on);

static const setting_group_detail_t aux_groups[] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t aux_settings[] = {
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL, Setting_NonCoreFn, aux_set_invert_out, aux_get_invert_out },
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, input_ports, NULL, NULL, Setting_NonCore, &settings.ioport.invert_in.mask, NULL, is_setting_available },
};

static setting_details_t details = {
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

static setting_details_t *on_get_settings (void)
{
    return &details;
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
    if(port < AUX_N_OUT) {

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
                    value = DIGITAL_IN(input->pin) ^ invert;
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
            if((DIGITAL_IN(input->pin) ^ invert) == wait_for) {
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

void ioports_event (input_signal_t *input)
{
    spin_lock = true;
    event_bits |= input->bit;

    if(input->interrupt_callback)
        input->interrupt_callback(input->id - Output_Aux0, !DIGITAL_IN(input->pin));

    spin_lock = false;
}

static int32_t wait_on_input (bool digital, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    if(digital) {
        if(port < n_in)
            value = get_input(&aux_in[port], (settings.ioport.invert_in.mask << port) & 0x01, wait_mode, timeout);
    }
//    else if(port == 0)
//        value = analogRead(41);

    return value;
}

static bool register_interrupt_handler (uint8_t port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr interrupt_callback)
{
    bool ok;

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

static void set_pin_description (bool digital, bool output, uint8_t port, const char *s)
{
    if(digital) {
        if(!output && port < n_in)
            aux_in[port].description = s;

 //       if(output && port < n_out)
 //           aux_out[port].description = s;
    }
}

void board_init (pin_group_pins_t *aux_inputs, output_sr_t *reg)
{
    sr = reg;
    aux_in = aux_inputs->pins.inputs;

    hal.port.digital_out = digital_out;
    hal.port.num_digital_out = n_out = AUX_N_OUT;

    if((hal.port.num_digital_in = n_in = aux_inputs->n_pins)) {
        hal.port.wait_on_input = wait_on_input;
        hal.port.register_interrupt_handler = register_interrupt_handler;
    }

    hal.port.set_pin_description = set_pin_description;

    details.on_get_settings = grbl.on_get_settings;
    grbl.on_get_settings = on_get_settings;

    uint_fast8_t i;

    for(i = 0; i < min(hal.port.num_digital_in, 8); i++) {
        strcat(input_ports, i == 0 ? "Port " : ",Port ");
        strcat(input_ports, uitoa(i));
    }
}

#endif
