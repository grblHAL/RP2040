/*
  picobob_map.h - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021-2023 Andrew Marles

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY// without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif
// Define ganged axis or A axis step pulse and step direction output pins.
 
 

#define BOARD_NAME "IhorSPicoCNC"
#define BOARD_URL "https://ibotz.info"
/*https://github.com/Expatria-Technologies/PicoBOB/blob/main/readme_images/default_pinout.png*/
// 0 txd0 1 rxd0
// 4 txd1 5 rxd1
//#define PROBE_PORT            GPIO_INPUT
//#define PROBE_PIN             2
//pins 0-10 26 -29



// Define stepper driver enable/disable output pin.
#define ENABLE_PORT             GPIO_OUTPUT
#define STEPPERS_ENABLE_PIN     9

// Define step direction output pins.
#define DIRECTION_PORT        GPIO_OUTPUT
#define DIRECTION_OUTMODE     GPIO_MAP
#define X_DIRECTION_PIN       10
#define Y_DIRECTION_PIN       11
#define Z_DIRECTION_PIN       12


// Define step pulse output pins.
#define STEP_PORT             GPIO_PIO  // N_AXIS pin PIO SM
#define STEP_PINS_BASE        13        // N_AXIS number of consecutive pins are used by PIO 13,14,15

// Define user-control controls (cycle start, reset, feed hold) input pins.  Only Estop is supported on the Mach3 BOB.
//#define CYCLE_START_PIN       16 //Resumes a job that is paused. Paired with "feed_hold_pin" it will allow a machine to be paused and resumed with physical buttons.
//#define FEED_HOLD_PIN         17 //Pauses a job that is running. Paired with "cycle_start_pin" it will allow a machine to be paused and resumed with physical buttons.

//Details: Resumes a job that is paused. Paired with "feed_hold_pin" it will allow a machine to be paused and resumed with physical buttons.

// Define homing/hard limit switch input pins.  Currently configured so that X and Z limit pins are shared.
#define LIMIT_PORT            GPIO_INPUT
#define X_LIMIT_PIN           21  
#define Y_LIMIT_PIN           20 
#define Z_LIMIT_PIN           19  
#define PROBE_PIN             18 
// $5=7 $6=1 $14=70  - invert probe hold start

// Define probe switch input pin.
#define AUXINPUT0_PIN           28
#define AUXOUTPUT0_PORT         GPIO_OUTPUT
#define AUXOUTPUT0_PIN          27
#define AUXOUTPUT1_PORT         GPIO_OUTPUT
#define AUXOUTPUT1_PIN          26
#if MODBUS_ENABLE
#define MODBUS_SERIAL_PORT          1
//#define MPG_STREAM          1
#endif




// 22  


//#define RESET_PIN             16
// #define ESTOP_PIN         22 //This can be used with an e-stop. A true e-stop should also cut the power


//$0 – Step pulse, microseconds
//$1 - Step idle delay, milliseconds\
//$2 – Step port invert, mask
//$3 – Direction port invert, mask
//$4 - Step enable invert, boolean
//$5 - Limit pins invert, boolean
//$6 - Probe pin invert, boolean
//$10 - Status report, mask
//$11 - Junction deviation, mm
//$12 – Arc tolerance, mm
//$13 - Report inches, boolean
//$20 - Soft limits, boolean
//$21 - Hard limits, boolean
//$22 - Homing cycle, boolean
//$23 - Homing dir invert, mask
//$24 - Homing feed, mm/min
//$25 - Homing seek, mm/min
//$27 - Homing pull-off, mm
//$30 - Max spindle speed, RPM
//$31 - Min spindle speed, RPM
//$32 - Laser mode, boolean
//$100, $101 and $102 – [X,Y,Z] steps/mm 320
//$110, $111 and $112 – [X,Y,Z] Max rate, mm/min 640
//$120, $121, $122 – [X,Y,Z] Acceleration, mm/sec^2
//$130, $131, $132 – [X,Y,Z] Max travel, mm

// $0=10.0                                                                         
// $1=25                                                                           
// $2=0                                                                            
// $3=0                                                                            
// $4=7                                                                            
// $5=7                                                                            
// $6=1                                                                            
// $10=511                                                                         
// $11=0.010                                                                       
// $12=0.002                                                                       
// $13=0                                                                           
// $14=0                                                                           
// $15=0                                                                           
// $16=0                                                                           
// $17=0                                                                           
// $18=0                                                                           
// $19=0                                                                           
// $20=0                                                                           
// $21=0                                                                           
// $22=1                                                                           
// $23=3                                                                           
// $24=25.0                                                                        
// $25=500.0                                                                       
// $26=250                                                                         
// $27=10.000                                                                      
// $28=0.100                                                                       
// $29=0.0                                                                         
// $30=24000.000                                                                   
// $31=6000.000                                                                    
// $32=0                                                                           
// $37=0                                                                           
// $39=1                                                                           
// $40=0                                                                           
// $43=1                                                                           
// $44=4                                                                           
// $45=3                                                                           
// $46=0                                                                           
// $50=100.0                                                                       
// $51=600.0                                                                       
// $52=3000.0                                                                      
// $53=0.250                                                                       
// $54=500.0                                                                       
// $55=3000.0                                                                      
// $62=0                                                                           
// $63=3                                                                           
// $64=0                                                                           
// $65=0                                                                           
// $70=35                                                                          
// $73=1                                                                           
// $74=IGORNET                                                                     
// $75=IG0RNET29041971                                                             
// $76=grbl                                                                        
// $77=grbl                                                                        
// $78=CA                                                                          
// $100=320.000                                                                    
// $101=320.000                                                                    
// $102=320.000                                                                    
// $110=640.000                                                                    
// $111=640.000                                                                    
// $112=640.000                                                                    
// $120=10.000                                                                     
// $121=10.000                                                                     
// $122=10.000                                                                     
// $130=200.000                                                                    
// $131=200.000                                                                    
// $132=200.000                                                                    
// $320=grblHAL                                                                    
// $322=192.168.5.1                                                                
// $323=192.168.5.1                                                                
// $324=255.255.255.0                                                              
// $325=23                                                                         
// $327=80                                                                         
// $337=                                                                           
// $340=0.0                                                                        
// $341=0                                                                          
// $342=30.0                                                                       
// $343=25.0                                                                       
// $344=200.0                                                                      
// $345=200.0                                                                      
// $346=1                                                                          
// $370=0                                                                          
// $372=0                                                                          
// $374=2                                                                          
// $375=50                                                                         
// $384=0                                                                          
// $394=4.0                                                                        
// $398=35                                                                         
// $460=1                                                                          
// $481=0                                                                          
// $486=0                                                                          
// $530=10.80.39.78                                                                
// $531=1883                                                                       
// $532=igor                                                                       
// $533=p29041971  



// $$ 15.09.2024
// $0=10.0
// $1=25
// $2=0
// $3=0
// $4=7
// $5=7
// $6=1
// $10=511
// $11=0.010
// $12=0.002
// $13=0                                                                           
// $14=6                                                                           
// $15=0                                                                           
// $17=0                                                                           
// $18=0                                                                           
// $19=0                                                                           
// $20=0                                                                           
// $21=0                                                                           
// $22=1                                                                           
// $23=6                                                                           
// $24=25.0                                                                        
// $25=1000.0                                                                      
// $26=250                                                                         
// $27=10.000                                                                      
// $28=0.100                                                                       
// $29=0.0                                                                         
// $30=24000.000                                                                   
// $31=6000.000                                                                    
// $32=0                                                                           
// $37=0                                                                           
// $39=1                                                                           
// $40=0                                                                           
// $43=1                                                                           
// $44=4                                                                           
// $45=3                                                                           
// $46=0                                                                           
// $50=100.0                                                                       
// $51=600.0                                                                       
// $52=3000.0                                                                      
// $53=0.250                                                                       
// $54=500.0                                                                       
// $55=3000.0                                                                      
// $62=0                                                                           
// $63=3                                                                           
// $64=0                                                                           
// $65=0                                                                           
// $70=35                                                                          
// $73=1                                                                           
// $74=                                                                            
// $75=                                                                            
// $76=grblHAL_AP                                                                  
// $77=grbl                                                                        
// $78=                                                                            
// $100=320.00000                                                                  
// $101=320.00000                                                                  
// $102=320.00000                                                                  
// $110=1000.000                                                                   
// $111=1000.000                                                                   
// $112=1000.000                                                                   
// $120=40.000                                                                     
// $121=10.000                                                                     
// $122=10.000                                                                     
// $130=846.000                                                                    
// $131=700.000                                                                    
// $132=500.000                                                                    
// $320=grblHAL                                                                    
// $322=10.80.39.130                                                               
// $323=10.80.39.3                                                                 
// $324=255.255.255.0                                                              
// $325=23                                                                         
// $327=80                                                                         
// $337=                                                                           
// $340=0.0                                                                        
// $341=0                                                                          
// $342=30.0                                                                       
// $343=25.0                                                                       
// $344=200.0                                                                      
// $345=200.0                                                                      
// $346=1                                                                          
// $370=0                                                                          
// $372=0                                                                          
// $374=2                                                                          
// $375=50                                                                         
// $384=0                                                                          
// $394=4.0                                                                        
// $398=35                                                                         
// $460=1                                                                          
// $481=0                                                                          
// $486=0                                                                          
// $530=0.0.0.0                                                                    
// $531=1883                                                                       
// $532=                                                                           
// $533=                                                                           
// ok 
