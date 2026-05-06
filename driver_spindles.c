/*

  driver_spindles.c - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021 Volksolive
  Copyright (c) 2021-2026 Terje Io

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

#if DRIVER_SPINDLE_ENABLE || DRIVER_SPINDLE1_ENABLE

#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "grbl/task.h"

#ifdef USE_EXPANDERS
extern xbar_t *iox_out[N_AUX_DOUT_MAX];
#endif

#if PPI_ENABLE

#include "laser/ppi.h"

static hal_timer_t ppi_timer;
static laser_ppi_t laser_ppi = {0};

static void __not_in_flash_func(PPI_TIMER_IRQHandler)(void)
{
    PPI_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag;

    laser_ppi.spindle_off(laser_ppi.spindle);
}

static void spindlePulseOn (spindle_ptrs_t *spindle, uint_fast16_t pulse_length)
{
    //    PPI_TIMER->ARR = pulse_length;
    //    PPI_TIMER->EGR = TIM_EGR_UG;
    //    PPI_TIMER->CR1 |= TIM_CR1_CEN;

    laser_ppi.spindle_on(spindle);
}

#endif // PPI_ENABLE

#if DRIVER_SPINDLE_ENABLE

static spindle_id_t spindle_id = -1;

// Static spindle (off, on cw & on ccw)

#if !(DRIVER_SPINDLE_ENABLE & (SPINDLE_ENA|SPINDLE_DIR)) // PWM only spindle

inline static void spindle_on (spindle_ptrs_t *spindle)
{
    spindle->context.pwm->flags.enable_out = On;
}

inline static void spindle_off (spindle_ptrs_t *spindle)
{
    spindle->context.pwm->flags.enable_out = Off;
}

inline static void spindle_dir (bool ccw)
{
    UNUSED(ccw);
}

#else

#ifndef SPINDLE_PORT
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_PORT SPINDLE_ENABLE_PORT
#else
#define SPINDLE_PORT SPINDLE_DIRECTION_PORT
#endif
#if (DRIVER_SPINDLE_ENABLE & SPINDLE_DIR) && SPINDLE_PORT != SPINDLE_DIRECTION_PORT
#error "Cannot mix port types for PWM0 enable and direction!"
#endif
#endif

#if SPINDLE_PORT == GPIO_OUTPUT

inline static void spindle_off (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    spindle->context.pwm->flags.enable_out = Off;
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    if(spindle->context.pwm->flags.cloned)
        DIGITAL_OUT(SPINDLE_DIRECTION_PIN, Off);
    else
        DIGITAL_OUT(SPINDLE_ENABLE_PIN, Off);
  #else
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, Off);
  #endif
#else
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, Off);
#endif
}

inline static void spindle_on (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    if(spindle->context.pwm->flags.cloned)
        DIGITAL_OUT(SPINDLE_DIRECTION_PIN, On);
    else
        DIGITAL_OUT(SPINDLE_ENABLE_PIN, On);
  #else
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, On);
  #endif
  #if SPINDLE_ENCODER_ENABLE
    if(!spindle->context.pwm->flags.enable_out && spindle->reset_data)
        spindle->reset_data();
  #endif
    spindle->context.pwm->flags.enable_out = On;
#else
    DIGITAL_OUT(SPINDLE_ENABLE_PIN, On);
#endif
}

inline static void spindle_dir (bool ccw)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    DIGITAL_OUT(SPINDLE_DIRECTION_PIN, ccw);
#else
    UNUSED(ccw);
#endif
}

#elif SPINDLE_PORT == EXPANDER_PORT

inline static void spindle_off (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    spindle->context.pwm->flags.enable_out = Off;
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    if(spindle->context.pwm->flags.cloned) {
        EXPANDER_OUT(SPINDLE_DIRECTION_PIN, settings.pwm_spindle.invert.ccw);
    } else {
        EXPANDER_OUT(SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
    }
  #else
    EXPANDER_OUT(SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
  #endif
#else
    EXPANDER_OUT(SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
#endif
}

inline static void spindle_on (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    if(spindle->context.pwm->flags.cloned) {
        EXPANDER_OUT(SPINDLE_DIRECTION_PIN, !settings.pwm_spindle.invert.ccw);
    } else {
        EXPANDER_OUT(SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
    }
  #else
    EXPANDER_OUT(SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
  #endif
  #if SPINDLE_ENCODER_ENABLE
    if(!spindle->context.pwm->flags.enable_out && spindle->reset_data)
        spindle->reset_data();
  #endif
    spindle->context.pwm->flags.enable_out = On;
#else
    EXPANDER_OUT(SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
#endif
}

inline static void spindle_dir (bool ccw)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    EXPANDER_OUT(SPINDLE_DIRECTION_PIN, ccw ^ settings.pwm_spindle.invert.ccw);
#else
    UNUSED(ccw);
#endif
}

#endif
#endif

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!state.on)
        spindle_off(spindle);
    else {
        spindle_dir(state.ccw);
        spindle_on(spindle);
    }
}

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

static settings_changed_ptr settings_changed;

// Variable spindle control functions

static void pwm_off (spindle_ptrs_t *spindle)
{
    if (spindle->context.pwm->flags.always_on)
        pwm_set_gpio_level(SPINDLE_PWM_PIN, spindle->context.pwm->off_value);
    else
        pwm_set_gpio_level(SPINDLE_PWM_PIN, spindle->context.pwm->off_value);
}

// Sets spindle speed
static void __not_in_flash_func(spindleSetSpeed)(spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if(pwm_value == spindle->context.pwm->off_value) {

        if(spindle->context.pwm->flags.rpm_controlled) {
            spindle_off(spindle);
            if(spindle->context.pwm->flags.laser_off_overdrive)
                pwm_set_gpio_level(SPINDLE_PWM_PIN, spindle->context.pwm->pwm_overdrive);
        } else
            pwm_off(spindle);

    } else {

        if(!spindle->context.pwm->flags.enable_out && spindle->context.pwm->flags.rpm_controlled)
            spindle_on(spindle);

        pwm_set_gpio_level(SPINDLE_PWM_PIN, pwm_value);
    }
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!(spindle->context.pwm->flags.cloned ? state.ccw : state.on)) {
        spindle_off(spindle);
        pwm_off(spindle);
    } else {
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
        if(!spindle->context.pwm->flags.cloned)
            spindle_dir(state.ccw);
#endif
        if(rpm == 0.0f && spindle->context.pwm->flags.rpm_controlled)
            spindle_off(spindle);
        else {
            spindle_on(spindle);
            spindleSetSpeed(spindle, spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false));
        }
    }
}

bool spindleConfig (spindle_ptrs_t *spindle)
{
    static spindle_pwm_t spindle_pwm;

    if (spindle == NULL)
        return false;

    uint32_t prescaler = 1, clock_hz = clock_get_hz(clk_sys);

    if(spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, clock_hz / prescaler)) {

        while(spindle_pwm.period > 65534 && prescaler < 255) {
            prescaler++;
            spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, clock_hz / prescaler);
        }

        while(spindle_pwm.period > 65534) {
            settings.pwm_spindle.pwm_freq += 1.0f;
            spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, clock_hz / prescaler);
        }

        spindle->set_state = spindleSetStateVariable;

#if PPI_ENABLE
        if(spindle_pwm.flags.laser_mode_disable) {
            if(laser_ppi.spindle == spindle)
                laser_ppi.spindle = NULL;
            spindle->pulse_on = NULL;
        } else if(ppi_timer) {
            laser_ppi.spindle = spindle;
            laser_ppi.spindle_on = spindle_on;
            laser_ppi.spindle_off = spindle_off;
            spindle->pulse_on = spindlePulseOn;
        }
#endif

        // Get the default config for
        pwm_config config = pwm_get_default_config();

        // Set divider, not using the 4 fractional bit part of the clock divider, only the integer part
        pwm_config_set_clkdiv_int(&config, prescaler);
        // Set the top value of the PWM => the period
        pwm_config_set_wrap(&config, spindle_pwm.period);
        // Set the off value of the PWM => off duty cycle (either 0 or the off value)
        pwm_set_gpio_level(SPINDLE_PWM_PIN, spindle_pwm.off_value);

        // Set polarity of the channel
    //    uint channel = pwm_gpio_to_channel(SPINDLE_PWM_PIN);                                                                                // Get which is associated with the PWM pin
    //    pwm_config_set_output_polarity(&config, (!channel & settings.pwm_spindle.invert.pwm), (channel & settings.pwm_spindle.invert.pwm)); // Set the polarity of the pin's channel

        // Load the configuration into our PWM slice, and set it running.
        pwm_init(pwm_gpio_to_slice_num(SPINDLE_PWM_PIN), &config, true);
    } else {
        if(spindle->context.pwm->flags.enable_out)
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        spindle->pulse_on = NULL;
        spindle->set_state = spindleSetState;
    }

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

static void settingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    settings_changed(settings, changed);

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    if(changed.spindle) {
        spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
        if(spindle_id == spindle_get_default())
            spindle_select(spindle_id);
    }
#endif
}

#endif // DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = { settings.pwm_spindle.invert.mask };

    UNUSED(spindle);

#if (DRIVER_SPINDLE_ENABLE & (SPINDLE_ENA|SPINDLE_DIR)) == 0 // PWM only spindle
    state.on = spindle->context.pwm->flags.enable_out ^ state.on;
#elif SPINDLE_PORT == GPIO_OUTPUT
    state.on = DIGITAL_IN(SPINDLE_ENABLE_PIN);
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    state.ccw = DIGITAL_IN(SPINDLE_DIRECTION_PIN);
  #endif
#elif SPINDLE_PORT == EXPANDER_PORT
    state.on = EXPANDER_IN(SPINDLE_ENABLE_PIN);
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    state.ccw = EXPANDER_IN(SPINDLE_DIRECTION_PIN);
  #endif
#endif

    state.value ^= settings.pwm_spindle.invert.mask;

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    state.on |= spindle->param->state.on;
#endif

#if SPINDLE_ENCODER_ENABLE
    if(spindle->get_data) {
        spindle_data_t *spindle_data = spindle->get_data(SpindleData_AtSpeed);
        state.at_speed = spindle_data->state_programmed.at_speed;
        state.encoder_error = spindle_data->state_programmed.encoder_error;
    }
#endif

    return state;
}

#endif // DRIVER_SPINDLE_ENABLE

#if DRIVER_SPINDLE1_ENABLE

static spindle_id_t spindle1_id = -1;
static spindle1_pwm_settings_t *spindle_config;

// Static spindle (off, on cw & on ccw)

#if !(DRIVER_SPINDLE1_ENABLE & (SPINDLE_ENA|SPINDLE_DIR)) // PWM only spindle

inline static void spindle1_on (spindle_ptrs_t *spindle)
{
    spindle->context.pwm->flags.enable_out = On;
}

inline static void spindle1_off (spindle_ptrs_t *spindle)
{
    spindle->context.pwm->flags.enable_out = Off;
}

inline static void spindle1_dir (bool ccw)
{
    UNUSED(ccw);
}

#else

#ifndef SPINDLE1_PORT
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_ENA
#define SPINDLE1_PORT SPINDLE1_ENABLE_PORT
#else
#define SPINDLE1_PORT SPINDLE1_DIRECTION_PORT
#endif
#if (DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR) && SPINDLE1_PORT != SPINDLE1_DIRECTION_PORT
#error "Cannot mix port types for PWM1 enable and direction!"
#endif
#endif

#if SPINDLE1_ENABLE_PORT == GPIO_OUTPUT

inline static void spindle1_off (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
    spindle->context.pwm->flags.enable_out = Off;
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
    if(spindle->context.pwm->flags.cloned)
        DIGITAL_OUT(SPINDLE1_DIRECTION_PIN, Off);
    else
        DIGITAL_OUT(SPINDLE1_ENABLE_PIN, Off);
  #else
    DIGITAL_OUT(SPINDLE1_ENABLE_PIN, Off);
  #endif
#else
    DIGITAL_OUT(SPINDLE1_ENABLE_PIN, Off);
#endif
}

inline static void spindle1_on (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
    if(spindle->context.pwm->flags.cloned)
        DIGITAL_OUT(SPINDLE1_DIRECTION_PIN, On);
    else
        DIGITAL_OUT(SPINDLE1_ENABLE_PIN, On);
  #else
    DIGITAL_OUT(SPINDLE1_ENABLE_PIN, On);
  #endif
  #if SPINDLE_ENCODER_ENABLE
    if(!spindle->context.pwm->flags.enable_out && spindle->reset_data)
        spindle->reset_data();
  #endif
    spindle->context.pwm->flags.enable_out = On;
#else
    DIGITAL_OUT(SPINDLE1_ENABLE_PIN, On);
#endif
}

inline static void spindle1_dir (bool ccw)
{
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
    DIGITAL_OUT(SPINDLE1_DIRECTION_PIN, ccw);
#else
    UNUSED(ccw);
#endif
}

#elif SPINDLE1_PORT == EXPANDER_PORT

inline static void spindle1_off (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
    spindle->context.pwm->flags.enable_out = Off;
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
    if(spindle->context.pwm->flags.cloned) {
        EXPANDER_OUT(SPINDLE1_DIRECTION_PIN, spindle_config->cfg.invert.ccw);
    } else {
        EXPANDER_OUT(SPINDLE1_ENABLE_PIN, spindle_config->cfg.invert.on);
    }
  #else
    EXPANDER_OUT(SPINDLE1_ENABLE_PIN, spindle_config->cfg.invert.on);
  #endif
#else
    EXPANDER_OUT(SPINDLE1_ENABLE_PIN, spindle_config->cfg.invert.on);
#endif
}

inline static void spindle1_on (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
    if(spindle->context.pwm->flags.cloned) {
        EXPANDER_OUT(SPINDLE1_DIRECTION_PIN, !spindle_config->cfg.invert.ccw);
    } else {
        EXPANDER_OUT(SPINDLE1_ENABLE_PIN, !spindle_config->cfg.invert.on);
    }
  #else
    EXPANDER_OUT(SPINDLE1_ENABLE_PIN, !spindle_config->cfg.invert.on);
  #endif
  #if SPINDLE_ENCODER_ENABLE
    if(!spindle->context.pwm->flags.enable_out && spindle->reset_data)
        spindle->reset_data();
  #endif
    spindle->context.pwm->flags.enable_out = On;
#else
    EXPANDER_OUT(SPINDLE1_ENABLE_PIN, !spindle_config->cfg.invert.on);
#endif
}

inline static void spindle1_dir (bool ccw)
{
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
    EXPANDER_OUT(SPINDLE1_DIRECTION_PIN, ccw ^ spindle_config->cfg.invert.ccw);
#else
    UNUSED(ccw);
#endif
}

#endif
#endif

// Start or stop spindle
static void spindle1SetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!state.on)
        spindle_off(spindle);
    else {
        spindle_dir(state.ccw);
        spindle_on(spindle);
    }
}

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM

// Variable spindle control functions

static void pwm1_off (spindle_ptrs_t *spindle)
{
    if (spindle->context.pwm->flags.always_on)
        pwm_set_gpio_level(SPINDLE1_PWM_PIN, spindle->context.pwm->off_value);
    else
        pwm_set_gpio_level(SPINDLE1_PWM_PIN, spindle->context.pwm->off_value);
}

// Sets spindle speed
static void __not_in_flash_func(spindle1SetSpeed)(spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if(pwm_value == spindle->context.pwm->off_value) {

        if(spindle->context.pwm->flags.rpm_controlled) {
            spindle1_off(spindle);
            if(spindle->context.pwm->flags.laser_off_overdrive)
                pwm_set_gpio_level(SPINDLE1_PWM_PIN, spindle->context.pwm->pwm_overdrive);
        } else
            pwm1_off(spindle);

    } else {

        if(!spindle->context.pwm->flags.enable_out && spindle->context.pwm->flags.rpm_controlled)
            spindle1_on(spindle);

        pwm_set_gpio_level(SPINDLE1_PWM_PIN, pwm_value);
    }
}

static uint_fast16_t spindle1GetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

// Start or stop spindle
static void spindle1SetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!(spindle->context.pwm->flags.cloned ? state.ccw : state.on)) {
        spindle1_off(spindle);
        pwm1_off(spindle);
    } else {
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
        if(!spindle->context.pwm->flags.cloned)
            spindle1_dir(state.ccw);
#endif
        if(rpm == 0.0f && spindle->context.pwm->flags.rpm_controlled)
            spindle1_off(spindle);
        else {
            spindle1_on(spindle);
            spindle1SetSpeed(spindle, spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false));
        }
    }
}

bool spindle1Config (spindle_ptrs_t *spindle)
{
    static spindle_pwm_t spindle_pwm;

    if (spindle == NULL)
        return false;

    uint32_t prescaler = 1, clock_hz = clock_get_hz(clk_sys);

    if(spindle_precompute_pwm_values(spindle, &spindle_pwm, &spindle_config->cfg, clock_hz / prescaler)) {

        while(spindle_pwm.period > 65534 && prescaler < 255) {
            prescaler++;
            spindle_precompute_pwm_values(spindle, &spindle_pwm, &spindle_config->cfg, clock_hz / prescaler);
        }

        while(spindle_pwm.period > 65534) {
            spindle_config->cfg.pwm_freq += 1.0f;
            spindle_precompute_pwm_values(spindle, &spindle_pwm, &spindle_config->cfg, clock_hz / prescaler);
        }

        spindle->set_state = spindle1SetStateVariable;

#if PPI_ENABLE
            if(spindle_pwm.flags.laser_mode_disable) {
                if(laser_ppi.spindle == spindle)
                    laser_ppi.spindle = NULL;
                spindle->pulse_on = NULL;
            } else if(laser_ppi.spindle == NULL && ppi_timer) {
                laser_ppi.spindle = spindle;
                laser_ppi.spindle_on = spindle1_on;
                laser_ppi.spindle_off = spindle1_off;
                spindle->pulse_on = spindlePulseOn;
            }
#endif

        // Get the default config for
        pwm_config config = pwm_get_default_config();

        // Set divider, not using the 4 fractional bit part of the clock divider, only the integer part
        pwm_config_set_clkdiv_int(&config, prescaler);
        // Set the top value of the PWM => the period
        pwm_config_set_wrap(&config, spindle_pwm.period);
        // Set the off value of the PWM => off duty cycle (either 0 or the off value)
        pwm_set_gpio_level(SPINDLE1_PWM_PIN, spindle_pwm.off_value);

        // Set polarity of the channel
    //    uint channel = pwm_gpio_to_channel(SPINDLE_PWM_PIN);                                                                                // Get which is associated with the PWM pin
    //    pwm_config_set_output_polarity(&config, (!channel & spindle_config->cfg.invert.pwm), (channel & settings.pwm_spindle.invert.pwm)); // Set the polarity of the pin's channel

        // Load the configuration into our PWM slice, and set it running.
        pwm_init(pwm_gpio_to_slice_num(SPINDLE1_PWM_PIN), &config, true);
    } else {
        if(spindle->context.pwm->flags.enable_out)
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        spindle->pulse_on = NULL;
        spindle->set_state = spindle1SetState;
    }

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

static void spindle1_settings_changed (spindle1_pwm_settings_t *settings)
{
    spindle1Config(spindle_get_hal(spindle1_id, SpindleHAL_Configured));
}

#endif // DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindle1GetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = { spindle_config->cfg.invert.mask };

    UNUSED(spindle);

#if !(DRIVER_SPINDLE1_ENABLE & (SPINDLE_ENA|SPINDLE_DIR)) // PWM only spindle
    state.on = spindle->context.pwm->flags.enable_out ^ state.on;
#elif SPINDLE1_PORT == GPIO_OUTPUT
    state.on = DIGITAL_IN(SPINDLE1_ENABLE_PIN);
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
    state.ccw = DIGITAL_IN(SPINDLE1_DIRECTION_PIN);
  #endif
#elif SPINDLE1_PORT == EXPANDER_PORT
    state.on = EXPANDER_IN(SPINDLE1_ENABLE_PIN);
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
    state.ccw = EXPANDER_IN(SPINDLE1_DIRECTION_PIN);
  #endif
#endif

    state.value ^= spindle_config->cfg.invert.mask;

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
    state.on |= spindle->param->state.on;
#endif

#if SPINDLE_ENCODER_ENABLE
    if(spindle->get_data) {
        spindle_data_t *spindle_data = spindle->get_data(SpindleData_AtSpeed);
        state.at_speed = spindle_data->state_programmed.at_speed;
        state.encoder_error = spindle_data->state_programmed.encoder_error;
    }
#endif

    return state;
}

#endif // DRIVER_SPINDLE1_ENABLE

void driver_spindles_init (void)
{
#if DRIVER_SPINDLE_ENABLE

 #if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_PWM0,
  #else
        .ref_id = SPINDLE_PWM0_NODIR,
  #endif
        .config = spindleConfig,
        .set_state = spindleSetStateVariable,
        .get_state = spindleGetState,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindleSetSpeed,
  #if PPI_ENABLE
        .pulse_on = spindlePulseOn,
  #endif
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
            .pwm_invert = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
            .enable = On,
  #endif
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
            .direction = On
  #endif
        }
    };

    settings_changed = hal.settings_changed;
    hal.settings_changed = settingsChanged;

 #elif DRIVER_SPINDLE_ENABLE & SPINDLE_ENA

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Basic,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_ONOFF1_DIR,
  #else
        .ref_id = SPINDLE_ONOFF1,
  #endif
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .cap = {
            .gpio_controlled = On,
            .enable = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR

            .direction = On
  #endif
        }
    };

 #endif

    spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME);

#endif // DRIVER_SPINDLE_ENABLE

#if DRIVER_SPINDLE1_ENABLE

 #if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM

    static const spindle_ptrs_t spindle1 = {
        .type = SpindleType_PWM,
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_PWM1,
  #else
        .ref_id = SPINDLE_PWM1_NODIR,
  #endif
        .config = spindle1Config,
        .set_state = spindle1SetStateVariable,
        .get_state = spindle1GetState,
        .get_pwm = spindle1GetPWM,
        .update_pwm = spindle1SetSpeed,
  #if PPI_ENABLE
        .pulse_on = spindle1PulseOn,
  #endif
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
            .pwm_invert = On,
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_ENA
            .enable = On,
  #endif
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
            .direction = On
  #endif
        }
    };
 
    if((spindle_config = spindle1_settings_add(false)) && (spindle1_id = spindle_register(&spindle1, DRIVER_SPINDLE1_NAME)) != -1)
        spindle1_settings_register(spindle1.cap, spindle1_settings_changed);
    else
        task_run_on_startup(report_warning, "PWM2 spindle failed to initialize!");

 #elif DRIVER_SPINDLE1_ENABLE & SPINDLE_ENA

    static const spindle_ptrs_t spindle1 = {
        .type = SpindleType_Basic,
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_ONOFF1_DIR,
  #else
        .ref_id = SPINDLE_ONOFF1,
  #endif
        .set_state = spindle1SetState,
        .get_state = spindle1GetState,
        .cap = {
            .gpio_controlled = On,
            .enable = On,
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
            .direction = On
  #endif
        }
    };

    if((spindle_config = spindle1_settings_add(false)) && (spindle1_id = spindle_register(&spindle1, DRIVER_SPINDLE1_NAME)) != -1)
       spindle1_settings_register(spindle1.cap, NULL);

 #endif
#endif // DRIVER_SPINDLE1_ENABLE

#if PPI_ENABLE
    ppi_init();
#endif
}

#endif // DRIVER_SPINDLE_ENABLE || DRIVER_SPINDLE1_ENABLE
