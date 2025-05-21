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

#include "flexgpio/flexgpio.h"
#include "boards/flexihal2350_map.h"

#define AUXOUTPUT0_PIN          23 //RP2040 pin
#define AUXOUTPUT1_PIN          22 //RP2040 pin
#define AUXOUTPUT2_PIN          21 //RP2040 pin
#define AUXOUTPUT3_PIN          20 //RP2040 pin
#define AUXOUTPUT4_PIN          19 //RP2040 pin
#define AUXOUTPUT5_PIN          18 //RP2040 pin
#define AUXOUTPUT6_PIN          17 //RP2040 pin
#define AUXOUTPUT7_PIN          16 //RP2040 pin

static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;
static on_reset_ptr on_reset;

static uint32_t debounce_ms = 0;
static uint32_t polling_ms = 0;
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

void board_ports_init(void) {
    // Initialize the FlexGPIO ports for the board
    uint_fast8_t idx;
    uint_fast8_t n_ports = sizeof(flexgpio_aux_out) / sizeof(xbar_t);
    pin_function_t aux_out_base = Output_Aux0;

    flexgpio_direction_mask = 0;
    flexgpio_polarity_mask = 0;
    flexgpio_enable_mask = 0xFFFFFFFF;

    for (idx = 0; idx < n_ports; idx++) {
        flexgpio_aux_out[idx].id = idx;        
        flexgpio_aux_out[idx].port = &flexgpio_outpins;

        //need to explicitly assign pin functions for the board as pins are not sequential
        switch (idx) {
            case 0:
                flexgpio_aux_out[idx].pin = AUXOUTPUT0_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 0;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;
            case 1:
                flexgpio_aux_out[idx].pin = AUXOUTPUT1_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 1;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;
            case 2:
                flexgpio_aux_out[idx].pin = AUXOUTPUT2_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 2;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;
            case 3:
                flexgpio_aux_out[idx].pin = AUXOUTPUT3_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 3;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;
            case 4:
                flexgpio_aux_out[idx].pin = AUXOUTPUT4_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 4;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;
            case 5:
                flexgpio_aux_out[idx].pin = AUXOUTPUT5_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 5;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;
            case 6:
                flexgpio_aux_out[idx].pin = AUXOUTPUT6_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 6;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;
            case 7:
                flexgpio_aux_out[idx].pin = AUXOUTPUT7_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 7;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;
            case 8:
                flexgpio_aux_out[idx].pin = SPINDLE_ENABLE_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 8;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;       
            case 9:
                flexgpio_aux_out[idx].pin = SPINDLE_DIRECTION_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 9;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;
            case 10:
                flexgpio_aux_out[idx].pin = COOLANT_FLOOD_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 10;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;     
            case 11:
                flexgpio_aux_out[idx].pin = COOLANT_MIST_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 11;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;  
            case 12:
                flexgpio_aux_out[idx].pin = X_ENABLE_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 12;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;   
            case 13:
                flexgpio_aux_out[idx].pin = Y_ENABLE_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 13;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;     
            case 14:
                flexgpio_aux_out[idx].pin = Z_ENABLE_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 14;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break;  
#ifdef M3_ENABLE_PIN            
                case 15:
                flexgpio_aux_out[idx].pin = M3_ENABLE_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 15;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break; 
#endif
#ifdef M4_ENABLE_PIN                     
            case 16:
                flexgpio_aux_out[idx].pin = M4_ENABLE_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 16;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break; 
#endif  
#ifdef M5_ENABLE_PIN                     
            case 17:
                flexgpio_aux_out[idx].pin = M5_ENABLE_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 17;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << idx);
                break; 
#endif                                                                                                                                                      
            default:
                flexgpio_aux_out[idx].id = idx;
                flexgpio_aux_out[idx].function = Input_Unassigned;
                flexgpio_aux_out[idx].group = PinGroup_Virtual;
                flexgpio_aux_out[idx].cap.output = Off;
                flexgpio_aux_out[idx].cap.external = Off;
                flexgpio_aux_out[idx].cap.claimable = Off;
                flexgpio_aux_out[idx].mode.output = Off;
                break;
        }//close switch statement
    }//close for statement
}



void board_init (void)
{
    //board_ports_init();
    
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
