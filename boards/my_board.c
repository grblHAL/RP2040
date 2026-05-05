// 文件名: boards/my_board.c

#include "driver.h"
#include "grbl/hal.h"
#include "plugins/pca9654e.h"
// 确保这段代码只在您的板卡配置下生效
#if defined(BOARD_PICOBOB_G540) // <-- 请确保这个宏与您的配置一致

// 引入 ioports 接口，这是与插件通信的标准方式
#include "grbl/ioports.h"

// 1. 定义我们自己的冷却液控制函数
static void my_coolant_set_state(coolant_state_t mode)
{

  //  hal.stream.write("你好 my_coolant_set_state " ASCII_EOL);

    // 调用标准的 ioports 接口来控制数字输出
    // 系统会自动找到由 pca9654e 注册的端口
    ioport_digital_out(COOLANT_FLOOD_PIN, mode.flood);
    ioport_digital_out(COOLANT_MIST_PIN, mode.mist);
}
// 1. 保存原始的 digital_out 函数指针
static digital_out_ptr original_digital_out = NULL;

// aux不附带的
static void my_central_digital_out(uint8_t port, bool on)
{
    //hal.stream.write("你好 my_central_digital_out " ASCII_EOL);

    // --- 开始修改 ---
    //hal.stream.write("你好 my_central_digital_out - 端口: ");
   // hal.stream.write(uitoa(port)); // 使用 uitoa 转换端口号为字符串
   // hal.stream.write(", 状态: ");
  //  hal.stream.write(on ? "ON\n" : "OFF\n");
    // --- 修改结束 ---

    /// 处理aux
    if (port == 0)
    {
        tca9555_set_pin(14, on);
    }
    else if (port == 1)
    {
        tca9555_set_pin(11, on);
    }
    else if (port == 2)
    {
        tca9555_set_pin(0, on);
    }
    else if (port == 3)
    {
        tca9555_set_pin(1, on);
    }
    else if (port == 4)
    {
        tca9555_set_pin(2, on);
    }
    else if (port == 5)
    {
        tca9555_set_pin(3, on);
    }
    else if (port == 6)
    {
        tca9555_set_pin(4, on);
    }
    else if (port == 7)
    {
        tca9555_set_pin(5, on);
    }
    else if (port == 8)
    {
        tca9555_set_pin(6, on);
    }
    else if (port == 9)
    {
        tca9555_set_pin(7, on);
    }
    // ## aux结束

    // M7 M8
    else if (port == 15)
    {
        tca9555_set_pin(15, on);
    }
    else if (port == 12)
    {
        tca9555_set_pin(12, on);
    }

    // ## M7 M8结束
}



// 2. 在板卡初始化时，用我们的新函数替换掉系统默认的函数
void board_init(void)
{

    tca9555_init();

   // hal.stream.write("你好 board_init " ASCII_EOL);

    // 替换aux输出的函数
    hal.port.digital_out = my_central_digital_out;

    // 替换冷却液的函数
    hal.coolant.set_state = my_coolant_set_state;

}

#endif