/*
  citoh_cx6000.c - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2022-2024 Terje Io

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

#ifdef BOARD_CITOH_CX6000

#include "pico/time.h"
#include "hardware/gpio.h"

#include "keypad/keypad.h"

#include "grbl/protocol.h"
#include "grbl/motion_control.h"

#include "hpgl/motori.h"

#define PEN_XPOS 10.0f
#define PEN_YPOS 37.5f
#define PEN_N 6
#define PEN_YINSERT 7.5f

static pen_status_t pen_status;
static bool debouncing = false, online = true;
uint8_t pen_down_port = 4, online_port = 5, pen_led_port = 0, alert_led_port = 1, panel_reset_port = 2;
uint8_t jog_ports[] = { 0, 1, 2 ,3};
uint8_t jog_map[] = { CMD_JOG_CANCEL, JOG_YB, JOG_YF, 0, JOG_XL, JOG_XLYB, JOG_XLYF, 0, JOG_XR, JOG_XRYB, JOG_XRYF};

static inline uint_fast8_t get_jog_input (void)
{
    uint_fast8_t idx = 4, map = 0;

    do {
        if(hal.port.wait_on_input(Port_Digital, jog_ports[--idx], WaitMode_Immediate, 0.0f))
          map |= 1;
        if(idx)
          map <<= 1;
    } while(idx);

    map ^= 0b1111;

    if(map < sizeof(jog_map))
        map = jog_map[map];
//    else
//        map = CMD_JOG_CANCEL;

    return map;
}

static int64_t jog_debounced (alarm_id_t id, void *map)
{
    if(debouncing) {

        char c = get_jog_input();

        if(c != CMD_JOG_CANCEL)
            keypad_enqueue_keycode(CMD_JOG_CANCEL);

        if(c && c == get_jog_input())
            keypad_enqueue_keycode(c);

        debouncing = false;
    }

    return 0;
}

static void jog_irq_handler (uint8_t port, bool state)
{
    static alarm_id_t id = 0;

 //   if(online)
 //       return;

    if(debouncing && get_jog_input() == 0) {
        debouncing = false;
        cancel_alarm(id);
        keypad_enqueue_keycode(CMD_JOG_CANCEL);
    } else if(!debouncing) {
        debouncing = true;
        id = add_alarm_in_ms(15, jog_debounced, NULL, true);
    }
}

static void set_pen_status (void *data)
{
    if(state_get() == STATE_IDLE)
        pen_control(pen_status);
}

static int64_t pen_changed (alarm_id_t id, void *map)
{
    online = hal.port.wait_on_input(Port_Digital, online_port, WaitMode_Immediate, 0.0f) == 0;
    pen_status = (online && hal.port.wait_on_input(Port_Digital, pen_down_port, WaitMode_Immediate, 0.0f) != 1) ? Pen_Down : Pen_Up;
    debouncing = false;
    if(pen_status != get_pen_status())
        protocol_enqueue_foreground_task(set_pen_status, NULL);
    
    return 0;
}

static void pen_irq_handler (uint8_t port, bool state)
{
    if(!debouncing) {
        debouncing = true;
        add_alarm_in_ms(40, pen_changed, NULL, true);
    }
}

static void online_irq_handler (uint8_t port, bool state)
{
    if(!debouncing) {
        debouncing = true;
        pen_status = Pen_Up;
        protocol_enqueue_foreground_task(set_pen_status, NULL);
        add_alarm_in_ms(40, pen_changed, NULL, true);
    }
}

static void register_handlers (void *data)
{
    hal.port.digital_out(panel_reset_port, 0);

    hal.port.register_interrupt_handler(jog_ports[0], IRQ_Mode_Change, jog_irq_handler);
    hal.port.register_interrupt_handler(jog_ports[1], IRQ_Mode_Change, jog_irq_handler);
    hal.port.register_interrupt_handler(jog_ports[2], IRQ_Mode_Change, jog_irq_handler);
    hal.port.register_interrupt_handler(jog_ports[3], IRQ_Mode_Change, jog_irq_handler);
    hal.port.register_interrupt_handler(pen_down_port, IRQ_Mode_Change, pen_irq_handler);
    hal.port.register_interrupt_handler(online_port, IRQ_Mode_Change, online_irq_handler);

    pen_control(Pen_Up);
    pen_status = get_pen_status();

    online = hal.port.wait_on_input(Port_Digital, online_port, WaitMode_Immediate, 0.0f) == 0;
}

void board_init (void)
{
    hal.port.claim(Port_Digital, Port_Output, &panel_reset_port, "Panel reset");
    hal.port.digital_out(panel_reset_port, 1);

    hal.port.claim(Port_Digital, Port_Output, &pen_led_port, "Pen LED");
    hal.port.claim(Port_Digital, Port_Output, &alert_led_port, "Alert LED");

    hal.port.claim(Port_Digital, Port_Input, &jog_ports[0], "Jog Y-");
    hal.port.claim(Port_Digital, Port_Input, &jog_ports[1], "Jog Y+");
    hal.port.claim(Port_Digital, Port_Input, &jog_ports[2], "Jog X-");
    hal.port.claim(Port_Digital, Port_Input, &jog_ports[3], "Jog X+");
    hal.port.claim(Port_Digital, Port_Input, &pen_down_port, "Pen up/down");
    hal.port.claim(Port_Digital, Port_Input, &online_port, "Online"); // active low

    protocol_enqueue_foreground_task(register_handlers, NULL); // delay interrupt enable until startup comple
}

// Implementation of weak HPGL interface functions 

bool is_plotter_online (void)
{
    return online;
}

void x_select_pen (uint_fast16_t pen)
{
    static uint_fast16_t current = 256;

    if(current == pen || pen > 6)
        return;

    plan_line_data_t plan_data = {0};
    coord_data_t target = {0}, *origin = get_origin();

    plan_data.feed_rate = 150.0f;
    plan_data.condition.rapid_motion = On;

    // return current pen

    if(current != 256) {
        target.x = origin->x + PEN_XPOS + 10.0f;
        target.y = origin->y + PEN_YPOS + 21.0f * current;

        if(mc_line(target.values, &plan_data)) {
            target.x -= 10.0f;
            plan_data.condition.rapid_motion = Off;
            mc_line(target.values, &plan_data);
            target.y += PEN_YINSERT;
            mc_line(target.values, &plan_data);
            target.x += 10.0f;
            mc_line(target.values, &plan_data);

            protocol_buffer_synchronize();
            sync_position();
        }
    }

    // fetch pen

    if(pen > 0) {

        plan_data.condition.rapid_motion = On;

        target.x = origin->x + PEN_XPOS + 10.0f;
        target.y = origin->y + PEN_YPOS + PEN_YINSERT + 21.0f * pen;

        if(mc_line(target.values, &plan_data)) {
            target.x -= 10.0f;
            plan_data.condition.rapid_motion = Off;
            mc_line(target.values, &plan_data);
            target.y -= PEN_YINSERT;
            mc_line(target.values, &plan_data);
            target.x += 10.0f;
            mc_line(target.values, &plan_data);

            protocol_buffer_synchronize();
            sync_position();
        }
    }

    current = pen;
}

void pen_led (bool on)
{
    hal.port.digital_out(pen_led_port, !on);
}

void alert_led (bool on)
{
    hal.port.digital_out(alert_led_port, on);
}

#endif
