/*
2025年6月1日 拉取提交日期：2025 年 5 月 30 日
2025年6月13日 更新至核心改动 提交日期：2025 年 6 月 12日
22
*/
//PI_CNC控制板的头文件
#define BOARD_NAME "RP23U5XBB"

//MPG 串口定义
#define SERIAL1_PORT 1
#ifdef SERIAL1_PORT
#define UART_1_RX_PIN           27
#define UART_1_TX_PIN           99
#endif
//###

// P_IO 脉冲 17-21
#define STEP_PORT               GPIO_PIO  
#define STEP_PINS_BASE          17        

// 轴方向
#define DIRECTION_PORT          GPIO_OUTPUT
#define X_DIRECTION_PIN         12
#define Y_DIRECTION_PIN         13
#define Z_DIRECTION_PIN         14
#define M3_DIRECTION_PIN        15
#define M4_DIRECTION_PIN        16
#define DIRECTION_OUTMODE       GPIO_SHIFT12

//限位组
#define X_LIMIT_PIN             23 
#define Y_LIMIT_PIN             24 
#define Z_LIMIT_PIN             25 
#define M3_LIMIT_PIN            26 
#define M4_LIMIT_PIN            30 
#define LIMIT_INMODE            GPIO_MAP

#define M3_AVAILABLE
#define M4_AVAILABLE

//轴 信号定义结束

//M3 M4 PWM M7 M8
#define SPINDLE_PORT            GPIO_OUTPUT
#define SPINDLE_ENABLE_PIN      47
#define SPINDLE_DIRECTION_PIN   46
#define SPINDLE_PWM_PIN         22

#define COOLANT_PORT            GPIO_OUTPUT

#define COOLANT_FLOOD_PIN  44
#define COOLANT_MIST_PIN   45
//##

//输入 启动 暂停 复位 急停  探测

#define RESET_PIN               AUXINPUT0_PIN

#define FEED_HOLD_PIN           AUXINPUT1_PIN

#define PROBE_PIN               AUXINPUT2_PIN

//启动36 复位 35 我自行处理
//##

#if SPINDLE_ENCODER_ENABLE
#define SPINDLE_PULSE_PIN       29  // Must be an odd pin
#define SPINDLE_INDEX_PIN       28
#endif

//输入 输出
 #define AUXOUTPUT0_PORT         GPIO_OUTPUT
 #define AUXOUTPUT0_PIN          43
 #define AUXOUTPUT1_PORT         GPIO_OUTPUT
 #define AUXOUTPUT1_PIN          42
 #define AUXOUTPUT2_PORT         GPIO_OUTPUT
 #define AUXOUTPUT2_PIN          41
 #define AUXOUTPUT3_PORT         GPIO_OUTPUT
 #define AUXOUTPUT3_PIN          40

 #define AUXINPUT0_PIN           38   //RESET
 #define AUXINPUT1_PIN           37   //HOLD
 #define AUXINPUT2_PIN           34   //PROBE

 #define AUXINPUT3_PIN           39
 #define AUXINPUT4_PIN           33
 #define AUXINPUT5_PIN           32
 #define AUXINPUT6_PIN           31

//我的添加
#define AUXINPUT23_PIN 36  //我的启动 AUXINPUT23_PIN 36号引脚
#define AUXINPUT22_PIN 35  //我的复位 AUXINPUT22_PIN 35号引脚

//我的添加end

#if SDCARD_ENABLE 

#define SPI_PORT                0
#define SPI_SCK_PIN             2
#define SPI_MOSI_PIN            3
#define SPI_MISO_PIN            4

#if SDCARD_ENABLE
#define SD_CS_PIN               5
#endif
#endif // SDCARD_ENABLE 


