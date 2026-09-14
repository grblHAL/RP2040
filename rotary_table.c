/*

  rotary_table.c - M102/M103/M104, stepper-driven rotary table independent of grbl's motion planner

  Part of grblHAL

  Copyright (c) 2026 Terje Io

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

  Usage:
    M102 [S<pulses/s>] [P<0|1>] - start continuous rotation (P0 = CW, P1 = CCW,
                                   defaults to the last used direction/rate).
    M103                        - stop rotation immediately (also cancels a
                                   pending M104 move).
    M104 Q<degrees> [S<pulses/s>] - rotate by <degrees> (signed, relative) at
                                   the given rate, then stop automatically.
                                   Negative angle = CCW.

  The table motor is driven directly via a free-running RP2040 PWM slice on
  the STEP pin (a plain constant-frequency square wave, direction/enable set
  through ordinary GPIOs) instead of through grbl's stepper segment buffer.
  This is deliberate: grbl's G-code motion is a single coordinated multi-axis
  planner where every block moves all axes together and blocks execute in
  strict order, so there is no way to have one axis spin indefinitely while
  X/Y/Z keep accepting and executing independent G-code. Driving the table
  entirely outside that planner means X/Y/Z motion is never blocked by it.

  M104's auto-stop is time-based (duration = steps / rate) rather than an
  exact pulse count, scheduled via task_add_delayed() so it doesn't block
  X/Y/Z execution either. This intentionally avoids the PWM wrap interrupt
  (PWM_IRQ_WRAP), which driver.c already claims exclusively for spindle RPM
  encoding when SPINDLE_ENCODER_ENABLE is on.
*/

#include "driver.h"

#if ROTARY_TABLE_ENABLE

#include <math.h>
#include <string.h>

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/task.h"

#ifndef ROTARY_TABLE_STEP_PIN
#define ROTARY_TABLE_STEP_PIN 14
#endif
#ifndef ROTARY_TABLE_DIR_PIN
#define ROTARY_TABLE_DIR_PIN 13
#endif
#ifndef ROTARY_TABLE_ENABLE_PIN
#define ROTARY_TABLE_ENABLE_PIN 15
#endif
#ifndef ROTARY_TABLE_ENABLE_ACTIVE_LOW
#define ROTARY_TABLE_ENABLE_ACTIVE_LOW 1 // TMC2209 EN is active low, matching this board's X/Y/Z wiring ($4 invert mask).
#endif
#ifndef ROTARY_TABLE_STEPS_PER_DEG
#define ROTARY_TABLE_STEPS_PER_DEG 8.88889f // Calibrated empirically (see README) - direct drive, no gearbox.
#endif
#ifndef ROTARY_TABLE_RATE_DEFAULT
#define ROTARY_TABLE_RATE_DEFAULT 100 // steps/s
#endif
#define ROTARY_TABLE_RATE_MIN 1
#define ROTARY_TABLE_RATE_MAX 20000

static uint pwm_slice, pwm_chan;
static uint32_t sys_clk_hz;
static uint32_t last_rate = ROTARY_TABLE_RATE_DEFAULT;
static bool last_ccw = false;
static volatile uint32_t move_seq = 0;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static driver_reset_ptr driver_reset;

static void table_enable (bool on)
{
    gpio_put(ROTARY_TABLE_ENABLE_PIN, ROTARY_TABLE_ENABLE_ACTIVE_LOW ? !on : on);
}

static void table_stop (void)
{
    pwm_set_enabled(pwm_slice, false);
    table_enable(false);
}

// Sets the PWM slice to a free-running square wave at the given frequency.
static void table_set_rate (uint32_t rate)
{
    float divider;
    uint32_t wrap;

    if(rate < ROTARY_TABLE_RATE_MIN)
        rate = ROTARY_TABLE_RATE_MIN;
    if(rate > ROTARY_TABLE_RATE_MAX)
        rate = ROTARY_TABLE_RATE_MAX;

    // Aim for a ~50000 count wrap for resolution, capped to the 8.4 fixed-point
    // clkdiv range (1.0 - 255.9375) and the 16-bit wrap register.
    divider = (float)sys_clk_hz / ((float)rate * 50000.0f);
    if(divider < 1.0f)
        divider = 1.0f;
    else if(divider > 255.9f)
        divider = 255.9f;

    wrap = (uint32_t)((float)sys_clk_hz / (divider * (float)rate)) - 1;
    if(wrap < 2)
        wrap = 2;
    else if(wrap > 65535)
        wrap = 65535;

    pwm_set_clkdiv(pwm_slice, divider);
    pwm_set_wrap(pwm_slice, wrap);
    pwm_set_chan_level(pwm_slice, pwm_chan, (wrap + 1) / 2); // ~50% duty cycle
}

static void table_start (uint32_t rate, bool ccw)
{
    pwm_set_enabled(pwm_slice, false);

    gpio_put(ROTARY_TABLE_DIR_PIN, ccw);
    table_enable(true);

    table_set_rate(rate);
    pwm_set_counter(pwm_slice, 0);
    pwm_set_enabled(pwm_slice, true);
}

static void table_scheduled_stop (void *data)
{
    if((uint32_t)(uintptr_t)data == move_seq)
        table_stop();
}

static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return (mcode == UserMCode_Generic2 || mcode == UserMCode_Generic3 || mcode == UserMCode_Generic4)
                     ? UserMCode_Normal
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t mcode_validate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    switch(gc_block->user_mcode) {

        case UserMCode_Generic2: // M102 - start continuous rotation
            if(gc_block->words.s && gc_block->values.s < 1.0f)
                state = Status_GcodeValueOutOfRange;
            if(state == Status_OK && gc_block->words.p &&
                (!isintf(gc_block->values.p) || (gc_block->values.p != 0.0f && gc_block->values.p != 1.0f)))
                state = Status_GcodeValueOutOfRange;
            gc_block->words.s = gc_block->words.p = Off;
            break;

        case UserMCode_Generic3: // M103 - stop
            break;

        case UserMCode_Generic4: // M104 - rotate by angle
            if(!gc_block->words.q)
                state = Status_GcodeValueWordMissing;
            else if(gc_block->words.s && gc_block->values.s < 1.0f)
                state = Status_GcodeValueOutOfRange;
            gc_block->words.q = gc_block->words.s = Off;
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    switch(gc_block->user_mcode) {

        case UserMCode_Generic2: { // M102 - start continuous rotation

            uint32_t rate = gc_block->words.s ? (uint32_t)gc_block->values.s : last_rate;
            bool ccw = gc_block->words.p ? gc_block->values.p != 0.0f : last_ccw;

            last_rate = rate;
            last_ccw = ccw;
            move_seq++; // invalidate any pending M104 auto-stop

            table_start(rate, ccw);

        } break;

        case UserMCode_Generic3: // M103 - stop
            move_seq++; // invalidate any pending M104 auto-stop
            table_stop();
            break;

        case UserMCode_Generic4: { // M104 - rotate by angle, then auto-stop

            uint32_t rate = gc_block->words.s ? (uint32_t)gc_block->values.s : last_rate;
            float angle = gc_block->values.q;
            bool ccw = angle < 0.0f;
            uint32_t steps = (uint32_t)(fabsf(angle) * ROTARY_TABLE_STEPS_PER_DEG + 0.5f);
            uint32_t duration_ms = (uint32_t)((float)steps / (float)rate * 1000.0f + 0.5f);

            last_rate = rate;
            last_ccw = ccw;
            move_seq++;

            if(steps > 0) {
                table_start(rate, ccw);
                task_add_delayed(table_scheduled_stop, (void *)(uintptr_t)move_seq, duration_ms);
            }

        } break;

        default:
            if(user_mcode.execute)
                user_mcode.execute(state, gc_block);
            break;
    }
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Rotary table", "0.02");
}

// Stop the table on any soft reset (Ctrl-X / realtime reset command) - it is
// driven independently of grbl's motion system, so grbl's own reset handling
// has no effect on it otherwise.
static void onDriverReset (void)
{
    move_seq++; // invalidate any pending M104 auto-stop
    table_stop();

    driver_reset();
}

void rotary_table_init (void)
{
    sys_clk_hz = clock_get_hz(clk_sys);

    gpio_init(ROTARY_TABLE_DIR_PIN);
    gpio_set_dir(ROTARY_TABLE_DIR_PIN, GPIO_OUT);
    gpio_put(ROTARY_TABLE_DIR_PIN, false);

    gpio_init(ROTARY_TABLE_ENABLE_PIN);
    gpio_set_dir(ROTARY_TABLE_ENABLE_PIN, GPIO_OUT);
    table_enable(false);

    gpio_set_function(ROTARY_TABLE_STEP_PIN, GPIO_FUNC_PWM);
    pwm_slice = pwm_gpio_to_slice_num(ROTARY_TABLE_STEP_PIN);
    pwm_chan = pwm_gpio_to_channel(ROTARY_TABLE_STEP_PIN);
    table_set_rate(last_rate);

    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = mcode_check;
    grbl.user_mcode.validate = mcode_validate;
    grbl.user_mcode.execute = mcode_execute;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    driver_reset = hal.driver_reset;
    hal.driver_reset = onDriverReset;
}

#endif // ROTARY_TABLE_ENABLE
