/*

  timers.c - driver code for RP2xxx ARM processors

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
*/


// TODO: add RP2350 slices

#include "driver.h"

#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

typedef struct {
    uint8_t slice;
    bool claimed;
    timer_cfg_t cfg;
    timer_cap_t cap;
    uint32_t freq_hz;
} dtimer_t;

static dtimer_t timers[NUM_PWM_SLICES] = {};

static dtimer_t *timer_get (uint slice)
{
    return slice < NUM_PWM_SLICES ? &timers[slice] : NULL;
}

timer_resolution_t timer_get_resolution (uint slice)
{
    dtimer_t *dtimer;

    return (timer_resolution_t)((dtimer = timer_get(slice)) ? dtimer->cap.resolution : 0);
}

timer_cap_t timer_get_cap (uint slice)
{
    dtimer_t *dtimer;

    return (dtimer = timer_get(slice)) ? dtimer->cap : (timer_cap_t){0};
}

hal_timer_t timer_claim (uint slice)
{
    bool claimed = false;
    dtimer_t *dtimer;

    if((claimed = (dtimer = timer_get(slice)) && !dtimer->claimed))
        dtimer->claimed = true;

    return claimed ? dtimer : NULL;
}

bool timer_is_claimed (uint slice)
{
    dtimer_t *dtimer = timer_get(slice);

    return dtimer && dtimer->claimed;
}

uint32_t timer_get_clock_hz (uint slice)
{
    dtimer_t *dtimer = timer_get(slice);

    return dtimer ? dtimer->freq_hz : 0;
}

uint32_t timer_clk_enable (uint slice)
{
    dtimer_t *dtimer = timer_get(slice);

    return dtimer ? dtimer->freq_hz : 0;
}

static void timer_int_handler (void)
{
    uint32_t irq = pwm_get_irq_status_mask();

    while(irq) {
        uint32_t slice = ffs(irq) - 1;

        pwm_clear_irq(slice);

        irq &= ~(1 << slice);

        if(timers[slice].cfg.single_shot)
            pwm_set_irq_enabled(slice, false);

        if(timers[slice].cfg.timeout_callback)
            timers[slice].cfg.timeout_callback(timers[slice].cfg.context);
    }
}

#if NUM_PWM_IRQS > 1
#endif

hal_timer_t timerClaim (timer_cap_t cap, uint32_t timebase)
{
    static bool init_ok = false;

    hal_timer_t t = NULL;
    uint_fast8_t idx, n_timers = NUM_PWM_SLICES;

    for(idx = 0; idx < n_timers; idx++) {
        if(!timers[idx].claimed && timers[idx].cap.resolution == cap.resolution) {

            uint32_t prescaler = (timers[idx].freq_hz * (timebase / 1000)) / 1000000 - 1;

            if(prescaler > 0 && prescaler <= 255) {

                t = &timers[idx];
                timers[idx].claimed = true;

                pwm_config pwm_config = pwm_get_default_config();
                pwm_config_set_clkdiv_int(&pwm_config, prescaler);
                pwm_init(timers[idx].slice, &pwm_config, false);
                pwm_clear_irq(timers[idx].slice);

                if(!init_ok) {
                    init_ok = true;
                    irq_set_exclusive_handler(PWM_IRQ_WRAP, timer_int_handler);
                    irq_set_priority(PWM_IRQ_WRAP, PICO_HIGHEST_IRQ_PRIORITY);
                    irq_set_enabled(PWM_IRQ_WRAP, true); // Enable PWM IRQ
                }
                break;
            }
        }
    }

    return t;
}

bool timerCfg (hal_timer_t timer, timer_cfg_t *cfg)
{
    if(timer)
        memcpy(&((dtimer_t *)timer)->cfg, cfg, sizeof(timer_cfg_t));

    return !!timer;
}

bool timerStart (hal_timer_t timer, uint32_t period)
{
    if(((dtimer_t *)timer)->cfg.single_shot || period != ((dtimer_t *)timer)->cfg.period) {

        ((dtimer_t *)timer)->cfg.period = period;

        pwm_set_enabled(((dtimer_t *)timer)->slice, false);
        pwm_set_wrap(((dtimer_t *)timer)->slice, --period);
        pwm_set_enabled(((dtimer_t *)timer)->slice, true);
        pwm_set_irq_enabled(((dtimer_t *)timer)->slice, true);
    }

    return true;
}

bool timerStop (hal_timer_t timer)
{
    pwm_set_enabled(((dtimer_t *)timer)->slice, false);

    return true;
}

void timers_init (void)
{
    for(int idx = 0; idx < NUM_PWM_SLICES; idx++)
    {
        timers[idx].slice = idx;
        timers[idx].cap.resolution = Timer_16bit;
        timers[idx].freq_hz = clock_get_hz(clk_sys);
    }
#if !SPINDLE_ENCODER_ENABLE
    hal.timer.claim = timerClaim;
    hal.timer.configure = timerCfg;
    hal.timer.start = timerStart;
    hal.timer.stop = timerStop;
#endif
}
