/*
  crash_report.c - 崩溃黑匣子(HardFault + 看门狗挂死双通道)

  工作流程:
    1) 主循环钩子周期性喂看门狗,并把 "心跳魔数 + 检查点 + LR" 写到 scratch
    2) 若进入 HardFault(空指针/野指针/非法跳转等):
         汇编桩从异常栈帧取出 PC/LR/SP/PSR/R0,写到 scratch,走 FAULT 类型,软复位
    3) 若主循环挂死(死锁/误关中断/无限循环):
         心跳停止 → 看门狗超时(约 5 秒)自动复位,scratch 中保留最后心跳内容
    4) 启动极早期:根据 scratch 内容判断 FAULT / HANG,持久化到 flash 最后一个扇区
    5) grbl 启动后,随 $I 查询自动把 [CRASH:...] 推送到上位机
*/

#include <stdio.h>
#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "hardware/regs/addressmap.h"
#include "pico/bootrom.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/system.h"

#include "crash_report.h"

// ==================== 配置 ====================

// 看门狗超时时间(毫秒)。主循环正常迭代间隔远小于此值,留足余量避免误杀
#define WATCHDOG_TIMEOUT_MS 5000

// flash 最后一个扇区(4K),littlefs 尾部已预留 32K 空闲,不冲突
#define CRASH_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

// Cortex-M33 故障状态寄存器
#define SCB_CFSR  (*(volatile uint32_t *)0xE000ED28)
#define SCB_HFSR  (*(volatile uint32_t *)0xE000ED2C)

// ==================== 内部状态 ====================

static const crash_record_t * const flash_crash_ptr =
    (const crash_record_t *)(XIP_BASE + CRASH_FLASH_OFFSET);

static on_report_options_ptr on_report_options_prev;
static on_execute_realtime_ptr on_execute_realtime_prev;
static on_execute_realtime_ptr on_execute_delay_prev;

// 运行期当前检查点(16 位,由 crash_report_checkpoint 更新)
static volatile uint16_t current_checkpoint = 0;

// ==================== HardFault 通道 ====================

// 异常入口:naked 汇编桩,从正确的栈取异常帧地址交给 C 函数
__attribute__((naked, used)) void isr_hardfault(void)
{
    __asm volatile (
        "tst   lr, #4             \n" // EXC_RETURN bit2: 0=MSP,1=PSP
        "ite   eq                 \n"
        "mrseq r0, msp            \n"
        "mrsne r0, psp            \n"
        "mov   r1, lr             \n"
        "b     crash_report_save_and_reboot \n"
    );
}

// 保存现场到 scratch 并触发软复位
// stack 指向异常自动压栈的 8 个字(R0/R1/R2/R3/R12/LR/PC/xPSR)
__attribute__((used)) void crash_report_save_and_reboot(uint32_t *stack, uint32_t exc_lr)
{
    watchdog_hw->scratch[0] = CRASH_MAGIC;       // FAULT 标志
    watchdog_hw->scratch[1] = stack[6];          // PC
    watchdog_hw->scratch[2] = stack[5];          // LR
    watchdog_hw->scratch[3] = (uint32_t)stack;   // SP
    watchdog_hw->scratch[4] = stack[7];          // xPSR
    watchdog_hw->scratch[5] = SCB_CFSR;
    watchdog_hw->scratch[6] = SCB_HFSR;
    watchdog_hw->scratch[7] = ((stack[0] & 0xFFFFu) << 16) | (exc_lr & 0xFFFFu);

    // 触发一次性软复位(通过看门狗立即复位,不是周期性看门狗)
    watchdog_reboot(0, 0, 0);
    while (1)
        __asm volatile ("wfi");
}

// ==================== 看门狗挂死通道 ====================

// 主循环喂狗钩子:每次实时任务执行时调用
static void feed_watchdog_realtime(sys_state_t state)
{
    // 写入心跳魔数 + 当前检查点 + 调用者 LR
    watchdog_hw->scratch[0] = HEARTBEAT_MAGIC;
    watchdog_hw->scratch[1] = current_checkpoint;
    watchdog_hw->scratch[2] = (uint32_t)__builtin_return_address(0);

    watchdog_update();

    on_execute_realtime_prev(state);
}

// 长延时中也要喂狗
static void feed_watchdog_delay(sys_state_t state)
{
    watchdog_hw->scratch[0] = HEARTBEAT_MAGIC;
    watchdog_hw->scratch[1] = current_checkpoint;
    watchdog_hw->scratch[2] = (uint32_t)__builtin_return_address(0);

    watchdog_update();

    on_execute_delay_prev(state);
}

void crash_report_checkpoint(uint32_t id)
{
    current_checkpoint = (uint16_t)(id & 0xFFFFu);
}

// ==================== 启动检查 ====================

// 启动极早期调用,必须在看门狗被重新开启之前读 scratch
void crash_report_startup_check(void)
{
    uint32_t tag = watchdog_hw->scratch[0];

    // 不是任何已知类型 → 正常上电/按键复位,什么都不做
    if (tag != CRASH_MAGIC && tag != HEARTBEAT_MAGIC)
        return;

    crash_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = CRASH_MAGIC;

    if (tag == CRASH_MAGIC) {
        // HardFault 类型
        rec.type  = CRASH_TYPE_FAULT;
        rec.pc    = watchdog_hw->scratch[1];
        rec.lr    = watchdog_hw->scratch[2];
        rec.sp    = watchdog_hw->scratch[3];
        rec.psr   = watchdog_hw->scratch[4];
        rec.cfsr  = watchdog_hw->scratch[5];
        rec.hfsr  = watchdog_hw->scratch[6];
        rec.r0    = (watchdog_hw->scratch[7] >> 16) & 0xFFFFu;
    } else {
        // 看门狗挂死类型:tag == HEARTBEAT_MAGIC 且本次是 watchdog reset
        // 主循环曾经跑过(写过心跳)但最终没再喂狗 → 挂死
        if (!watchdog_caused_reboot()) {
            // 心跳还在但不是看门狗复位?理论上走不到(可能用户拔电断电),保守当做正常
            watchdog_hw->scratch[0] = 0;
            return;
        }
        rec.type       = CRASH_TYPE_HANG;
        rec.checkpoint = watchdog_hw->scratch[1];
        rec.last_lr    = watchdog_hw->scratch[2];
    }

    // 从老记录继承序号
    if (flash_crash_ptr->magic == CRASH_MAGIC)
        rec.sequence = flash_crash_ptr->sequence + 1;
    else
        rec.sequence = 1;

    // 清 scratch 标志,避免重复写入
    watchdog_hw->scratch[0] = 0;

    // 按 flash 页对齐
    static uint8_t page_buf[FLASH_PAGE_SIZE] __attribute__((aligned(4)));
    memset(page_buf, 0xFF, FLASH_PAGE_SIZE);
    memcpy(page_buf, &rec, sizeof(rec));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(CRASH_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CRASH_FLASH_OFFSET, page_buf, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

// ==================== 对外接口 ====================

bool crash_report_has_pending(void)
{
    return flash_crash_ptr->magic == CRASH_MAGIC;
}

const crash_record_t *crash_report_get(void)
{
    return flash_crash_ptr;
}

void crash_report_emit(void)
{
    if (!crash_report_has_pending())
        return;

    const crash_record_t *r = flash_crash_ptr;
    char msg[240];

    if (r->type == CRASH_TYPE_FAULT) {
        snprintf(msg, sizeof(msg),
            "[CRASH:type=FAULT,seq=%lu,pc=0x%08lX,lr=0x%08lX,sp=0x%08lX,psr=0x%08lX,cfsr=0x%08lX,hfsr=0x%08lX,r0=0x%04lX]\r\n",
            (unsigned long)r->sequence,
            (unsigned long)r->pc,
            (unsigned long)r->lr,
            (unsigned long)r->sp,
            (unsigned long)r->psr,
            (unsigned long)r->cfsr,
            (unsigned long)r->hfsr,
            (unsigned long)r->r0);
    } else if (r->type == CRASH_TYPE_HANG) {
        snprintf(msg, sizeof(msg),
            "[CRASH:type=HANG,seq=%lu,cp=%lu,lastlr=0x%08lX]\r\n",
            (unsigned long)r->sequence,
            (unsigned long)r->checkpoint,
            (unsigned long)r->last_lr);
    } else {
        return;
    }

    if (hal.stream.write)
        hal.stream.write(msg);
}

void crash_report_clear(void)
{
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(CRASH_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

// grbl 报告钩子:每次 $I 查询时追加崩溃信息
static void on_report_options_crash(bool newopt)
{
    on_report_options_prev(newopt);

    if (!newopt)
        crash_report_emit();
}

// ==================== 测试命令 ====================

// $CRASH - 关中断后死循环,模拟"整个系统挂死"(用来测试看门狗挂死通道)
static status_code_t cmd_crash(sys_state_t state, char *args)
{
    (void)state; (void)args;

    // 先打个检查点,方便区分是哪段代码挂的
    crash_report_checkpoint(9999);

    // 关中断 + 死循环
    __asm volatile ("cpsid i" ::: "memory");
    for (;;) {
        __asm volatile ("nop");
    }

    return Status_OK;
}

static const sys_command_t crash_command_list[] = {
    {"CRASH", cmd_crash, { .noargs = On }, { .str = "trigger a test crash" } },
};

static sys_commands_t crash_commands = {
    .n_commands = sizeof(crash_command_list) / sizeof(sys_command_t),
    .commands = crash_command_list
};

// ==================== 安装 ====================

void crash_report_install(void)
{
    // 报告钩子
    on_report_options_prev = grbl.on_report_options;
    grbl.on_report_options = on_report_options_crash;

    // 主循环喂狗钩子
    on_execute_realtime_prev = grbl.on_execute_realtime;
    grbl.on_execute_realtime = feed_watchdog_realtime;

    on_execute_delay_prev = grbl.on_execute_delay;
    grbl.on_execute_delay = feed_watchdog_delay;

    // 注册测试命令
    system_register_commands(&crash_commands);

    // 最后才开看门狗:此时 scratch 启动检查已完成,主循环钩子已装好
    // 第二参数 pause_on_debug=1:调试器暂停时看门狗也暂停,调试方便
    watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);
}
