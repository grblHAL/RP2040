/*
my_plugin.c - 统一按键处理版本（启动+复位+AUXINPUT0-15）
使用统一的配置表驱动架构，简化维护和扩展
*/
#include "grbl/hal.h"
#include "grbl/task.h"
#include "grbl/ioports.h"
#include "driver.h"
#include <stdio.h>

// ==================== 统一按键系统配置 ====================

// 按键ID枚举 - 统一管理所有按键
typedef enum {
    BUTTON_ID_START = 0,     // 启动按键 (AUXINPUT23)
    BUTTON_ID_RESET = 1,     // 复位按键 (AUXINPUT22)
    BUTTON_ID_AUX0 = 2,      // AUXINPUT0
    BUTTON_ID_AUX1 = 3,      // AUXINPUT1
    BUTTON_ID_AUX2 = 4,      // AUXINPUT2
    BUTTON_ID_AUX3 = 5,      // AUXINPUT3
    BUTTON_ID_AUX4 = 6,      // AUXINPUT4
    BUTTON_ID_AUX5 = 7,      // AUXINPUT5
    BUTTON_ID_AUX6 = 8,      // AUXINPUT6
    BUTTON_ID_AUX7 = 9,      // AUXINPUT7
    BUTTON_ID_AUX8 = 10,     // AUXINPUT8
    BUTTON_ID_AUX9 = 11,     // AUXINPUT9
    BUTTON_ID_AUX10 = 12,    // AUXINPUT10
    BUTTON_ID_AUX11 = 13,    // AUXINPUT11
    BUTTON_ID_AUX12 = 14,    // AUXINPUT12
    BUTTON_ID_AUX13 = 15,    // AUXINPUT13
    BUTTON_ID_AUX14 = 16,    // AUXINPUT14
    BUTTON_ID_AUX15 = 17,    // AUXINPUT15
    BUTTON_ID_COUNT = 18     // 按键总数
} button_id_t;

// 按键配置结构体
typedef struct {
    uint8_t pin;                    // GPIO引脚号
    const char *message_prefix;     // 消息前缀
    uint8_t aux_port_num;          // AUXINPUT端口号（用于消息格式化）
    bool is_auxinput_format;       // 是否使用AUXINPUT格式（#AUXINPUT%d）
} button_config_t;

// 统一防抖控制结构体
static struct {
    uint32_t last_trigger_time[BUTTON_ID_COUNT];  // 每个按键的上次触发时间
    bool is_processing[BUTTON_ID_COUNT];          // 每个按键是否正在处理中
} unified_debounce;

// 防抖时间设置（毫秒）
#define DEBOUNCE_TIME_MS 50   // 50ms防抖时间

// 按键配置表 - 集中管理所有按键配置
static const button_config_t button_configs[BUTTON_ID_COUNT] = {
    // 启动和复位按键
    {AUXINPUT23_PIN, "anjian-qidong", 23, false},  // BUTTON_ID_START
    {AUXINPUT22_PIN, "anjian-fuwei",  22, false},  // BUTTON_ID_RESET

    // AUXINPUT0-15按键
    {AUXINPUT0_PIN,  "AUXINPUT", 0,  true},        // BUTTON_ID_AUX0
    {AUXINPUT1_PIN,  "AUXINPUT", 1,  true},        // BUTTON_ID_AUX1
    {AUXINPUT2_PIN,  "AUXINPUT", 2,  true},        // BUTTON_ID_AUX2
    {AUXINPUT3_PIN,  "AUXINPUT", 3,  true},        // BUTTON_ID_AUX3
    {AUXINPUT4_PIN,  "AUXINPUT", 4,  true},        // BUTTON_ID_AUX4
    {AUXINPUT5_PIN,  "AUXINPUT", 5,  true},        // BUTTON_ID_AUX5
    {AUXINPUT6_PIN,  "AUXINPUT", 6,  true},        // BUTTON_ID_AUX6
    {AUXINPUT7_PIN,  "AUXINPUT", 7,  true},        // BUTTON_ID_AUX7
    {AUXINPUT8_PIN,  "AUXINPUT", 8,  true},        // BUTTON_ID_AUX8
    {AUXINPUT9_PIN,  "AUXINPUT", 9,  true},        // BUTTON_ID_AUX9
    {AUXINPUT10_PIN, "AUXINPUT", 10, true},        // BUTTON_ID_AUX10
    {AUXINPUT11_PIN, "AUXINPUT", 11, true},        // BUTTON_ID_AUX11
    {AUXINPUT12_PIN, "AUXINPUT", 12, true},        // BUTTON_ID_AUX12
    {AUXINPUT13_PIN, "AUXINPUT", 13, true},        // BUTTON_ID_AUX13
    {AUXINPUT14_PIN, "AUXINPUT", 14, true},        // BUTTON_ID_AUX14
    {AUXINPUT15_PIN, "AUXINPUT", 15, true},        // BUTTON_ID_AUX15
};

// 所有按键的信号结构体数组
static input_signal_t button_signals[BUTTON_ID_COUNT];

// 用于传递按键ID的数据结构
typedef struct {
    button_id_t button_id;
} button_msg_data_t;

// ==================== 统一消息发送和中断处理 ====================

// 通用按键消息发送函数 - 前台任务
static void send_button_message(void *data) {
    button_msg_data_t *msg_data = (button_msg_data_t *)data;
    button_id_t button_id = msg_data->button_id;

    // 参数有效性检查
    if (button_id >= BUTTON_ID_COUNT) {
        return;
    }

    const button_config_t *config = &button_configs[button_id];
    char message_buffer[64];
    uint32_t timestamp_ms = hal.get_elapsed_ticks();

    // 根据配置决定消息格式
    if (config->is_auxinput_format) {
        // AUXINPUT格式: #AUXINPUT<port_num>,t=<timestamp>
        snprintf(message_buffer, sizeof(message_buffer), "#%s%d,t=%lu\r\n",
                 config->message_prefix, config->aux_port_num, timestamp_ms);
    } else {
        // 普通格式: #<message_prefix>,t=<timestamp>
        snprintf(message_buffer, sizeof(message_buffer), "#%s,t=%lu\r\n",
                 config->message_prefix, timestamp_ms);
    }

    hal.stream.write_all(message_buffer);

    // 发送完成后，清除处理标志
    unified_debounce.is_processing[button_id] = false;
}

// 通用按键中断处理函数 - 中断回调
static void generic_button_handler(uint8_t port_num, bool state) {
    uint32_t current_time = hal.get_elapsed_ticks();

    // 只在按钮按下时处理（低电平触发）
    if (state == false) {
        // 根据port_num查找对应的button_id
        button_id_t button_id = port_num;  // port_num就是button_id

        // 参数有效性检查
        if (button_id >= BUTTON_ID_COUNT) {
            return;
        }

        // 防抖检查：如果距离上次触发时间太短，或者正在处理中，则忽略
        if (unified_debounce.is_processing[button_id] ||
            (current_time - unified_debounce.last_trigger_time[button_id]) < DEBOUNCE_TIME_MS) {
            return;  // 忽略这次触发
        }

        // 记录这次触发时间并设置处理标志
        unified_debounce.last_trigger_time[button_id] = current_time;
        unified_debounce.is_processing[button_id] = true;

        // 创建消息数据（使用静态数组，因为任务是异步执行的）
        static button_msg_data_t msg_data[BUTTON_ID_COUNT];
        msg_data[button_id].button_id = button_id;

        // 添加任务到队列
        task_add_immediate(send_button_message, &msg_data[button_id]);
    }
}

// ==================== 初始化函数 ====================

// 完整的初始化函数 - 使用循环统一配置所有按键
void my_plugin_init(void) {
    // 初始化机械坐标记忆功能 - Author: Claude Opus 4.1 (Anthropic AI Assistant) - Date: 2025-01-10
    mem_position_init();
    // 机械坐标记忆功能初始化结束

    // 初始化统一防抖控制
    for (int i = 0; i < BUTTON_ID_COUNT; i++) {
        unified_debounce.last_trigger_time[i] = 0;
        unified_debounce.is_processing[i] = false;
    }

    // 获取外部引用
    extern input_signal_t *irq_pins[];
    extern void gpio_int_handler(uint gpio, uint32_t events);

    // 循环配置所有18个按键
    for (button_id_t button_id = 0; button_id < BUTTON_ID_COUNT; button_id++) {
        const button_config_t *config = &button_configs[button_id];
        input_signal_t *signal = &button_signals[button_id];

        // 初始化信号结构体
        if (button_id == BUTTON_ID_START) {
            signal->id = Input_Aux23;
        } else if (button_id == BUTTON_ID_RESET) {
            signal->id = Input_Aux22;
        } else {
            signal->id = Input_Aux0 + (button_id - BUTTON_ID_AUX0);  // Input_Aux0 到 Input_Aux15
        }

        signal->pin = config->pin;
        signal->user_port = button_id;  // 使用button_id作为user_port，便于在中断中识别
        signal->group = PinGroup_AuxInput;
        signal->port = GPIO_INPUT;
        signal->interrupt_callback = generic_button_handler;

        // 手动初始化GPIO引脚
        gpio_init(signal->pin);
        gpio_set_dir(signal->pin, GPIO_IN);

        // 配置上拉和反转
        signal->mode.pull_mode = PullMode_Up;
        signal->mode.inverted = false;
        gpio_set_pulls(signal->pin, true, false);
        gpio_set_inover(signal->pin, GPIO_OVERRIDE_NORMAL);

        // 注册到主中断分发器
        gpio_set_irq_enabled_with_callback(signal->pin,
                                           GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                                           true, &gpio_int_handler);

        // 将信号结构体添加到中断分发器的查找数组中
        irq_pins[signal->pin] = signal;
    }
}
