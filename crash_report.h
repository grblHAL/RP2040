/*
  crash_report.h - HardFault 黑匣子,记录崩溃现场并持久化到 flash
*/

#ifndef _CRASH_REPORT_H_
#define _CRASH_REPORT_H_

#include <stdint.h>
#include <stdbool.h>

// 崩溃类型
typedef enum {
    CRASH_TYPE_NONE  = 0,
    CRASH_TYPE_FAULT = 1,  // HardFault(空指针/野指针/非法跳转等)
    CRASH_TYPE_HANG  = 2,  // 看门狗超时(死锁/关中断死循环等)
} crash_type_t;

// 崩溃记录结构(写入 flash 最后一个扇区的第一页)
typedef struct {
    uint32_t magic;        // 魔数,为 CRASH_MAGIC 时记录有效
    uint32_t sequence;     // 累计崩溃次数
    uint32_t type;         // crash_type_t
    uint32_t pc;           // 崩溃指令地址(addr2line 用)
    uint32_t lr;           // 链接寄存器(上一级调用地址)
    uint32_t sp;           // 栈指针
    uint32_t psr;          // xPSR
    uint32_t cfsr;         // Configurable Fault Status(仅 FAULT 有效)
    uint32_t hfsr;         // HardFault Status(仅 FAULT 有效)
    uint32_t r0;           // R0
    uint32_t checkpoint;   // 最后检查点编号(HANG 类型时重要)
    uint32_t last_lr;      // 最后心跳时 LR(HANG 类型时主循环位置)
    uint32_t reserved[4];  // 预留,保持 64 字节
} crash_record_t;

#define CRASH_MAGIC 0xDEADC0DEu

// 运行期心跳标记(写入 scratch[0],用于看门狗复位后区分挂死)
#define HEARTBEAT_MAGIC 0xA11EFEEDu

// 启动早期调用:若 scratch 中有崩溃现场,复制到 flash
void crash_report_startup_check(void);

// 是否有未清除的崩溃记录
bool crash_report_has_pending(void);

// 拿到崩溃记录只读指针
const crash_record_t *crash_report_get(void);

// 通过 hal.stream.write 发 [CRASH:...] 消息
void crash_report_emit(void);

// 擦除 flash 中的崩溃记录
void crash_report_clear(void);

// 安装 grbl 报告钩子 + 心跳喂狗钩子 + 开启看门狗
void crash_report_install(void);

// 在关键代码位置打检查点,看门狗触发时最后一次检查点会记录到崩溃日志
// 参数范围 1~65535,0 保留
void crash_report_checkpoint(uint32_t id);

#endif
