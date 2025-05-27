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
#include "grbl/system.h"

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

static on_settings_changed_ptr on_settings_changed;
static on_state_change_ptr on_state_change;
static on_reset_ptr on_reset;

#define N_MOTOR_ALARMS 6

static int8_t val[N_MOTOR_ALARMS] = {0};
static bool motor_alarm_active = false;

static axes_signals_t motor_alarm_pins;

static void alarm_reset (void)
{
    if(on_reset)
        on_reset();

    motor_alarm_active = false;
}

static void execute_alarm (sys_state_t state)
{   

    if(!motor_alarm_active){

        uint32_t pins  = flexgpio_read_inputs();

        motor_alarm_pins.x = (pins >> X_ALARM_PIN) & 1U;
        motor_alarm_pins.y = (pins >> Y_ALARM_PIN) & 1U;
        motor_alarm_pins.z = (pins >> Z_ALARM_PIN) & 1U;
        motor_alarm_pins.a = (pins >> M3_ALARM_PIN) & 1U;
        motor_alarm_pins.b = (pins >> M4_ALARM_PIN) & 1U;
        motor_alarm_pins.c = (pins >> M5_ALARM_PIN) & 1U;

        if(motor_alarm_pins.x){
            report_message("Motor Error on X Motor!", Message_Warning);   
        }

        if(motor_alarm_pins.y){
            report_message("Motor Error on Y Motor!", Message_Warning);   
        }
        
        if(motor_alarm_pins.z){
            report_message("Motor Error on Z Motor!", Message_Warning);   
        }

        if(motor_alarm_pins.a){
            report_message("Motor Error on A Motor!", Message_Warning);   
        }

        if(motor_alarm_pins.b){
            report_message("Motor Error on B Motor!", Message_Warning);   
        }

        if(motor_alarm_pins.c){
          report_message("Motor Error on C Motor!", Message_Warning);   
      }        
        motor_alarm_active = true;
    }         
    
}

static void check_alarm_state (sys_state_t state)
{
    if(on_state_change)
        on_state_change(state);

    if (state == STATE_ALARM){
        if (sys.alarm == Alarm_MotorFault)
            execute_alarm (state);
    }
}

static inline uint32_t set_bit_cond(uint32_t mask, bool cond, uint8_t pin) {
    return cond ? (mask | (1 << pin)) : (mask & ~(1 << pin));
}

static void flexgpio_update_pins (void){

    // Initialize the FlexGPIO ports for the board
    uint_fast8_t idx;
    uint_fast8_t n_ports = sizeof(flexgpio_aux_out) / sizeof(xbar_t);
    pin_function_t aux_out_base = Output_Aux0;

    flexgpio_direction_mask = 0;
    flexgpio_polarity_mask = 0;
    flexgpio_enable_mask =  0; 

#if MOTOR_FAULT_ENABLE
    flexgpio_enable_mask   = set_bit_cond(flexgpio_enable_mask,   settings.motor_fault_enable.x, X_ALARM_PIN);
    flexgpio_polarity_mask = set_bit_cond(flexgpio_polarity_mask, settings.motor_fault_invert.x,  X_ALARM_PIN);

    flexgpio_enable_mask   = set_bit_cond(flexgpio_enable_mask,   settings.motor_fault_enable.y, Y_ALARM_PIN);
    flexgpio_polarity_mask = set_bit_cond(flexgpio_polarity_mask, settings.motor_fault_invert.y,  Y_ALARM_PIN);

    flexgpio_enable_mask   = set_bit_cond(flexgpio_enable_mask,   settings.motor_fault_enable.z, Z_ALARM_PIN);
    flexgpio_polarity_mask = set_bit_cond(flexgpio_polarity_mask, settings.motor_fault_invert.z,  Z_ALARM_PIN);

    #if N_ABC_MOTORS > 0
    flexgpio_enable_mask   = set_bit_cond(flexgpio_enable_mask,   settings.motor_fault_enable.a,  M3_ALARM_PIN);
    flexgpio_polarity_mask = set_bit_cond(flexgpio_polarity_mask, settings.motor_fault_invert.a,  M3_ALARM_PIN);
    #endif

    #if N_ABC_MOTORS >= 2
    flexgpio_enable_mask   = set_bit_cond(flexgpio_enable_mask,   settings.motor_fault_enable.b,  M4_ALARM_PIN);
    flexgpio_polarity_mask = set_bit_cond(flexgpio_polarity_mask, settings.motor_fault_invert.b,  M4_ALARM_PIN);
    #endif

    #if N_ABC_MOTORS >= 2
    flexgpio_enable_mask   = set_bit_cond(flexgpio_enable_mask,   settings.motor_fault_enable.b,  M5_ALARM_PIN);
    flexgpio_polarity_mask = set_bit_cond(flexgpio_polarity_mask, settings.motor_fault_invert.b,  M5_ALARM_PIN);
    #endif        
#endif

#if PROBE_ENABLE
    flexgpio_enable_mask   = set_bit_cond(flexgpio_enable_mask,   1,  PROBE_EXPANDER_PIN);
    flexgpio_polarity_mask = set_bit_cond(flexgpio_polarity_mask, settings.probe.invert_probe_pin,  PROBE_EXPANDER_PIN);

    //need to make the inversion match the GRBLHAL setting.  I think this is just the easiest way to handle this.
    flexgpio_polarity_mask = set_bit_cond(flexgpio_polarity_mask, settings.probe.invert_probe_pin,  MCU_PROBE_PIN);

    flexgpio_enable_mask   = set_bit_cond(flexgpio_enable_mask,   1,  TOOL_EXPANDER_PIN);
    flexgpio_polarity_mask = set_bit_cond(flexgpio_polarity_mask, settings.probe.invert_toolsetter_input,  TOOL_EXPANDER_PIN);    
#else
    flexgpio_enable_mask   = set_bit_cond(flexgpio_enable_mask,   0,  PROBE_EXPANDER_PIN);

    flexgpio_enable_mask   = set_bit_cond(flexgpio_enable_mask,   0,  TOOL_EXPANDER_PIN);   
#endif

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
                flexgpio_direction_mask |= (1U << AUXOUTPUT0_PIN);
                flexgpio_enable_mask |= (1U << AUXOUTPUT0_PIN);
                break;
            case 1:
                flexgpio_aux_out[idx].pin = AUXOUTPUT1_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 1;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << AUXOUTPUT1_PIN);
                flexgpio_enable_mask |= (1U << AUXOUTPUT1_PIN);
                break;
            case 2:
                flexgpio_aux_out[idx].pin = AUXOUTPUT2_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 2;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << AUXOUTPUT2_PIN);
                flexgpio_enable_mask |= (1U << AUXOUTPUT2_PIN);
                break;
            case 3:
                flexgpio_aux_out[idx].pin = AUXOUTPUT3_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 3;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << AUXOUTPUT3_PIN);
                flexgpio_enable_mask |= (1U << AUXOUTPUT3_PIN);
                break;
            case 4:
                flexgpio_aux_out[idx].pin = AUXOUTPUT4_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 4;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << AUXOUTPUT4_PIN);
                flexgpio_enable_mask |= (1U << AUXOUTPUT4_PIN);
                break;
            case 5:
                flexgpio_aux_out[idx].pin = AUXOUTPUT5_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 5;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << AUXOUTPUT5_PIN);
                flexgpio_enable_mask |= (1U << AUXOUTPUT5_PIN);
                break;
            case 6:
                flexgpio_aux_out[idx].pin = AUXOUTPUT6_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 6;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << AUXOUTPUT6_PIN);
                flexgpio_enable_mask |= (1U << AUXOUTPUT6_PIN);
                break;
            case 7:
                flexgpio_aux_out[idx].pin = AUXOUTPUT7_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 7;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_direction_mask |= (1U << AUXOUTPUT7_PIN);
                flexgpio_enable_mask |= (1U << AUXOUTPUT7_PIN);
                break;
            case 8:
                flexgpio_aux_out[idx].pin = SPINDLE_ENABLE_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 8;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_aux_out[idx].description = "Spindle Enable";
                flexgpio_direction_mask |= (1U << SPINDLE_ENABLE_PIN);
                flexgpio_enable_mask |= (1U << SPINDLE_ENABLE_PIN);
                break;       
            case 9:
                flexgpio_aux_out[idx].pin = SPINDLE_DIRECTION_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 9;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_aux_out[idx].description = "Spindle Direction";                
                flexgpio_direction_mask |= (1U << SPINDLE_DIRECTION_PIN);
                flexgpio_enable_mask |= (1U << SPINDLE_DIRECTION_PIN);
                break;
            case 10:
                flexgpio_aux_out[idx].pin = COOLANT_FLOOD_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 10;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_aux_out[idx].description = "Flood Coolant";                
                flexgpio_direction_mask |= (1U << COOLANT_FLOOD_PIN);
                flexgpio_enable_mask |= (1U << COOLANT_FLOOD_PIN);
                break;     
            case 11:
                flexgpio_aux_out[idx].pin = COOLANT_MIST_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 11;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_aux_out[idx].description = "Mist Coolant";                
                flexgpio_direction_mask |= (1U << COOLANT_MIST_PIN);
                flexgpio_enable_mask |= (1U << COOLANT_MIST_PIN);
                break;  
            case 12:
                flexgpio_aux_out[idx].pin = X_ENABLE_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 12;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_aux_out[idx].description = "X Stepper Enable";                
                flexgpio_direction_mask |= (1U << X_ENABLE_PIN);
                flexgpio_enable_mask |= (1U << X_ENABLE_PIN);
                break;   
            case 13:
                flexgpio_aux_out[idx].pin = Y_ENABLE_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 13;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_aux_out[idx].description = "Y Stepper Enable";                
                flexgpio_direction_mask |= (1U << Y_ENABLE_PIN);
                flexgpio_enable_mask |= (1U << Y_ENABLE_PIN);
                break;     
            case 14:
                flexgpio_aux_out[idx].pin = Z_ENABLE_PIN;
                flexgpio_aux_out[idx].function = aux_out_base + 14;
                flexgpio_aux_out[idx].group = PinGroup_AuxOutput;
                flexgpio_aux_out[idx].cap.output = On;
                flexgpio_aux_out[idx].cap.external = On;
                flexgpio_aux_out[idx].cap.claimable = On;
                flexgpio_aux_out[idx].mode.output = On;
                flexgpio_aux_out[idx].description = "Z Stepper Enable";                
                flexgpio_direction_mask |= (1U << Z_ENABLE_PIN);
                flexgpio_enable_mask |= (1U << Z_ENABLE_PIN);
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
                flexgpio_aux_out[idx].description = "A Stepper Enable";                
                flexgpio_direction_mask |= (1U << M3_ENABLE_PIN);
                flexgpio_enable_mask |= (1U << M3_ENABLE_PIN);
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
                flexgpio_aux_out[idx].description = "B Stepper Enable";                
                flexgpio_direction_mask |= (1U << M4_ENABLE_PIN);
                flexgpio_enable_mask |= (1U << M4_ENABLE_PIN);
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
                flexgpio_aux_out[idx].description = "C Stepper Enable";                
                flexgpio_direction_mask |= (1U << M5_ENABLE_PIN);
                flexgpio_enable_mask |= (1U << M5_ENABLE_PIN);
                break; 
#endif                                                                                                                                                      
            default:
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

static void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    if(on_settings_changed)
        on_settings_changed(settings, changed);

    flexgpio_update_pins();
    flexgpio_write_config();
}

void board_ports_init(void) {
    flexgpio_update_pins();

#if 1
    
    
#if MOTOR_FAULT_ENABLE
    on_reset = grbl.on_reset;
    grbl.on_reset = alarm_reset;

    on_state_change = grbl.on_state_change;
    grbl.on_state_change = check_alarm_state;

#endif

    on_settings_changed = grbl.on_settings_changed;
    grbl.on_settings_changed = onSettingsChanged;

#endif    
}

void board_init (void)
{    

    #if 1
    
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

    //flexgpio_update_pins();
    
    /*if((alm_nvs_address = nvs_alloc(sizeof(motor_alarm_settings_t)))) {
        settings_register(&setting_details);
    }  */

    hal.driver_cap.toolsetter = 1;

    hal.motor_fault_cap.a.x = 1;
    hal.motor_fault_cap.a.y = 1;
    hal.motor_fault_cap.a.z = 1;
    hal.motor_fault_cap.a.a = 1;
    hal.motor_fault_cap.a.b = 1;
    hal.motor_fault_cap.a.c = 1;
}




#endif
