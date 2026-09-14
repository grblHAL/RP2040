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

  Every "?" realtime status report gets two extra fields:
    |TBL:<angle 0-360>|TBLABS:<absolute angle, unwrapped, can exceed 360>
  Both are open-loop estimates (time elapsed x rate, no feedback sensor -
  same assumption grbl's own MPos makes for a stepper axis) and update live
  during an M102/M104 move, not just after it completes/auto-stops.
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
#include "grbl/report.h"
#include "grbl/nvs_buffer.h"
#include "motors/trinamic.h"

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

// The table's TMC2209 sits on the same shared UART bus as X/Y/Z (tmc_uart.c)
// but, being outside N_AXIS, is never touched by grbl's own Trinamic driver
// setup - left at power-on defaults it takes its microstep resolution from
// the MS1/MS2 *pins* (used for UART address selection on this board, not
// microstepping) instead of the UART-configured MRES register, which is far
// coarser than X/Y/Z's 16 microsteps and causes rough/jerky rotation
// (worst at low step rates, where individual coarse steps become visible).
// Explicitly adding it as a TMC2209 motor below fixes this the same way
// grbl's own settings load does for X/Y/Z.
#ifndef ROTARY_TABLE_TMC_ADDRESS
#define ROTARY_TABLE_TMC_ADDRESS 3 // UART address (was address_map[3] in tmc_uart.c when this motor was axis A).
#endif
#ifndef ROTARY_TABLE_TMC_MOTOR_ID
#define ROTARY_TABLE_TMC_MOTOR_ID 3 // tmc2209hal.c's tmcdriver[] slot - must not collide with X=0/Y=1/Z=2.
#endif
#ifndef ROTARY_TABLE_TMC_MICROSTEPS
#define ROTARY_TABLE_TMC_MICROSTEPS 16 // Matches X/Y/Z ($150-152).
#endif
#ifndef ROTARY_TABLE_TMC_CURRENT_MAX
#define ROTARY_TABLE_TMC_CURRENT_MAX 1500 // mA RMS. Motor is a 42BYGH34, rated 1500mA max - $453 is capped here.
#endif
#ifndef ROTARY_TABLE_TMC_CURRENT_DEFAULT
#define ROTARY_TABLE_TMC_CURRENT_DEFAULT 1000 // mA RMS, ~67% of max for thermal headroom.
#endif
#ifndef ROTARY_TABLE_TMC_HOLD_PCT
#define ROTARY_TABLE_TMC_HOLD_PCT 50 // % of run current while holding.
#endif
#ifndef ROTARY_TABLE_TMC_RSENSE
#define ROTARY_TABLE_TMC_RSENSE 110 // mOhm, TMC2209 sense resistor on this board.
#endif
#define ROTARY_TABLE_RATE_MIN 1
#define ROTARY_TABLE_RATE_MAX 20000

// $453-$454, see grbl/settings.h - reserved for private/local plugins ($450-452 are st3215.c's).
#define Setting_RotaryTable_Current      Setting_UserDefined_3
#define Setting_RotaryTable_ChopperMode  Setting_UserDefined_4

typedef struct {
    uint16_t current;      // mA RMS
    uint8_t spreadcycle;    // 0 = StealthChop, 1 = SpreadCycle
} rotary_table_settings_t;

static uint pwm_slice, pwm_chan;
static uint32_t sys_clk_hz;
static uint32_t last_rate = ROTARY_TABLE_RATE_DEFAULT;
static bool last_ccw = false;
static volatile uint32_t move_seq = 0;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static on_realtime_report_ptr on_realtime_report;
static driver_reset_ptr driver_reset;
static nvs_address_t nvs_address;
static rotary_table_settings_t rotary_table_settings;
static const tmchal_t *tmc_driver = NULL;

// Open-loop absolute position, degrees (unwrapped - keeps accumulating past
// +-360). No feedback sensor, same assumption grbl's own MPos makes for a
// normal stepper axis: correct as long as no steps are lost.
static float table_position_deg = 0.0f;
static bool table_spinning = false;
static uint32_t spin_start_tick = 0;
static uint32_t spin_rate = 0;
static bool spin_ccw = false;

static void table_enable (bool on)
{
    gpio_put(ROTARY_TABLE_ENABLE_PIN, ROTARY_TABLE_ENABLE_ACTIVE_LOW ? !on : on);
}

// Degrees turned since spin_start_tick at spin_rate/spin_ccw.
static float table_spin_delta (void)
{
    uint32_t elapsed_ms = hal.get_elapsed_ticks() - spin_start_tick;
    float delta = (float)elapsed_ms / 1000.0f * (float)spin_rate / ROTARY_TABLE_STEPS_PER_DEG;

    return spin_ccw ? -delta : delta;
}

// Current absolute position, folding in an in-progress spin.
static float table_current_position (void)
{
    return table_position_deg + (table_spinning ? table_spin_delta() : 0.0f);
}

static void table_stop (void)
{
    if(table_spinning) {
        table_position_deg += table_spin_delta();
        table_spinning = false;
    }

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

    spin_start_tick = hal.get_elapsed_ticks();
    spin_rate = rate;
    spin_ccw = ccw;
    table_spinning = true;
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

static void apply_tmc_settings (void)
{
    if(tmc_driver) {
        tmc_driver->set_current(ROTARY_TABLE_TMC_MOTOR_ID, rotary_table_settings.current, ROTARY_TABLE_TMC_HOLD_PCT);
        tmc_driver->stealthChop(ROTARY_TABLE_TMC_MOTOR_ID, !rotary_table_settings.spreadcycle);
    }
}

static status_code_t set_current (setting_id_t id, uint_fast16_t value)
{
    rotary_table_settings.current = (uint16_t)value;
    apply_tmc_settings();

    return Status_OK;
}

static uint32_t get_current (setting_id_t id)
{
    return rotary_table_settings.current;
}

static status_code_t set_chopper_mode (setting_id_t id, uint_fast16_t value)
{
    rotary_table_settings.spreadcycle = (uint8_t)value;
    apply_tmc_settings();

    return Status_OK;
}

static uint32_t get_chopper_mode (setting_id_t id)
{
    return rotary_table_settings.spreadcycle;
}

static const setting_detail_t rotary_table_settings_detail[] = {
    // max_value string must match ROTARY_TABLE_TMC_CURRENT_MAX above.
    { Setting_RotaryTable_Current, Group_General, "Rotary table motor current", "mA", Format_Int16, "####0", "0",
      "1500", Setting_IsExtendedFn, set_current, get_current, NULL },
    { Setting_RotaryTable_ChopperMode, Group_General, "Rotary table chopper mode", NULL, Format_RadioButtons,
      "StealthChop (quiet),SpreadCycle (louder & more torque)", NULL, NULL, Setting_IsExtendedFn, set_chopper_mode, get_chopper_mode, NULL }
};

static const setting_descr_t rotary_table_settings_descr[] = {
    { Setting_RotaryTable_Current, "Rotary table stepper motor RMS current. Check the motor's rated current before raising this - overcurrent will overheat it." },
    { Setting_RotaryTable_ChopperMode, "TMC2209 chopper algorithm for the rotary table motor. StealthChop is quieter but can whine, especially at low M102/M104 speeds; SpreadCycle is louder but often smoother." }
};

static void rotary_table_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&rotary_table_settings, sizeof(rotary_table_settings_t), true);
}

static void rotary_table_settings_restore (void)
{
    rotary_table_settings.current = ROTARY_TABLE_TMC_CURRENT_DEFAULT;
    rotary_table_settings.spreadcycle = 0; // StealthChop

    rotary_table_settings_save();
}

static void rotary_table_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&rotary_table_settings, nvs_address, sizeof(rotary_table_settings_t), true) != NVS_TransferResult_OK)
        rotary_table_settings_restore();

    if(rotary_table_settings.current > ROTARY_TABLE_TMC_CURRENT_MAX || rotary_table_settings.spreadcycle > 1)
        rotary_table_settings_restore();

    apply_tmc_settings();
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    float abs_angle = table_current_position();
    float wrapped = fmodf(abs_angle, 360.0f);
    char buf[48];

    if(wrapped < 0.0f)
        wrapped += 360.0f;

    strcpy(buf, "|TBL:");
    strcat(buf, ftoa(wrapped, 2));
    strcat(buf, "|TBLABS:");
    strcat(buf, ftoa(abs_angle, 2));
    stream_write(buf);

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Rotary table", "0.04");
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

#if TRINAMIC_ENABLE == 2209
    {
        motor_map_t motor = { .id = ROTARY_TABLE_TMC_MOTOR_ID, .axis = ROTARY_TABLE_TMC_MOTOR_ID };

        tmc_driver = TMC2209_AddMotor(motor, ROTARY_TABLE_TMC_ADDRESS, ROTARY_TABLE_TMC_CURRENT_DEFAULT,
                                       ROTARY_TABLE_TMC_MICROSTEPS, ROTARY_TABLE_TMC_RSENSE);

        if(tmc_driver)
            tmc_driver->set_microsteps(ROTARY_TABLE_TMC_MOTOR_ID, ROTARY_TABLE_TMC_MICROSTEPS);
        else
            task_run_on_startup(report_warning, "Rotary table: TMC2209 UART init failed, using driver defaults!");
    }
#endif

    {
        static setting_details_t setting_details = {
            .settings = rotary_table_settings_detail,
            .n_settings = sizeof(rotary_table_settings_detail) / sizeof(setting_detail_t),
            .descriptions = rotary_table_settings_descr,
            .n_descriptions = sizeof(rotary_table_settings_descr) / sizeof(setting_descr_t),
            .save = rotary_table_settings_save,
            .load = rotary_table_settings_load,
            .restore = rotary_table_settings_restore
        };

        // Deliberately not calling rotary_table_settings_load() here: doing so
        // hung the board during board_init() (confirmed by testing), likely
        // because it can trigger a flash write (via restore()) this early.
        // grbl's settings framework calls setting_details.load on its own at
        // a safe point later in boot, and TMC2209_AddMotor above already left
        // the driver in a safe working state (default current/StealthChop)
        // in the meantime.
        if((nvs_address = nvs_alloc(sizeof(rotary_table_settings_t))))
            settings_register(&setting_details);
        else {
            rotary_table_settings.current = ROTARY_TABLE_TMC_CURRENT_DEFAULT;
            rotary_table_settings.spreadcycle = 0;
        }
    }

    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = mcode_check;
    grbl.user_mcode.validate = mcode_validate;
    grbl.user_mcode.execute = mcode_execute;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_realtime_report = grbl.on_realtime_report;
    grbl.on_realtime_report = onRealtimeReport;

    driver_reset = hal.driver_reset;
    hal.driver_reset = onDriverReset;
}

#endif // ROTARY_TABLE_ENABLE
