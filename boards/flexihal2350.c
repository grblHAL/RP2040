/*
  flexihal2350.c - init code for flexihal2350

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

#include "driver.h"

#if defined(BOARD_FLEXIHAL2350)

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"
#include "grbl/nvs_buffer.h"

static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;
static on_reset_ptr on_reset;

static uint32_t debounce_ms = 0;
static uint32_t polling_ms = 0;
#define DEBOUNCE_DELAY 25
#define ALARM_THRESHOLD 3

#define N_MOTOR_ALARMS 5

static int8_t val[N_MOTOR_ALARMS] = {0};
static bool motor_alarm_active = false;

typedef struct {
    axes_signals_t     enable;
    axes_signals_t     invert;
} motor_alarm_settings_t;

static axes_signals_t motor_alarm_pins;

static nvs_address_t alm_nvs_address;
motor_alarm_settings_t motor_alarms;

static const setting_detail_t motor_alarm_settings[] = {
    { 744, Group_Stepper, "Motor Alarm enable", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExpanded, &motor_alarms.enable.mask, NULL, NULL},
    { 745, Group_Stepper, "Motor Alarm invert", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExpanded, &motor_alarms.invert.mask, NULL, NULL},
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t motor_alarm_descriptions[] = {
    { 744, "Enables the motor alarm" },
    { 745, "Inverts motor alarm signal" },
};

#endif
static void alarm_reset (void)
{
    if(on_reset)
        on_reset();

    motor_alarm_active = false;
}

static void execute_alarm (sys_state_t state)
{   
    
    if(!motor_alarm_active){

        if(motor_alarm_pins.x){
            report_message("Motor Error on X Axis!", Message_Warning);   
        }

        if(motor_alarm_pins.y){
            report_message("Motor Error on Y Axis!", Message_Warning);   
        }
        
        if(motor_alarm_pins.z){
            report_message("Motor Error on Z Axis!", Message_Warning);   
        }

        if(motor_alarm_pins.a){
            report_message("Motor Error on A Axis!", Message_Warning);   
        }

        if(motor_alarm_pins.b){
            report_message("Motor Error on B Axis!", Message_Warning);   
        }

        if(motor_alarm_pins.c){
          report_message("Motor Error on C Axis!", Message_Warning);   
      }        

        if(motor_alarm_pins.value > 0){
            system_set_exec_alarm(Alarm_EStop);
        }
        motor_alarm_active = true;
    }         
    
}


void board_init (void)
{
    /*
    settings.motor_fault_enable
    settings.motor_fault_invert
    */
    #if 1
    hal.driver_cap.toolsetter = 1;
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(SPI_MOSI_PIN, 1);
    gpio_put(SPI_MOSI_PIN,1);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(SPI_MISO_PIN, 1);
    gpio_put(SPI_MISO_PIN,1);  
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(SPI_SCK_PIN, 1);
    gpio_put(SPI_SCK_PIN,1);      
    gpio_set_function(SD_CS_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(SD_CS_PIN, 1);
    gpio_put(SD_CS_PIN,1);
    gpio_set_function(33, GPIO_FUNC_SIO);
    gpio_set_dir(33, 1);
    gpio_put(33,1);


    volatile uint32_t dly = 1000;
    volatile uint32_t count = 100;

    while(--dly)
      tight_loop_contents();
    
      while(--count) {
        gpio_put(SPI_SCK_PIN,1);        
        dly = 100;  // Reset dly before first delay
        while(--dly)    
          tight_loop_contents();

            gpio_put(SPI_SCK_PIN,0);        
        dly = 100;  // Reset dly before second delay
        while(--dly)
          tight_loop_contents();;
      }

    //sdcard_getfs(); // Mounts SD card if not already mounted      
    #endif

}




#endif
