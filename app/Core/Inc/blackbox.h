/**
  ******************************************************************************
  * @file    blackbox.h
  * @brief   黑匣子：W25Q 外部 Flash 上的事件环形日志（掉电不丢）
  ******************************************************************************
  * 用途：记录"故障发生/清除、会话切换、解锁、参数修改"等关键事件，
  * 类似车上的 EDR（Event Data Recorder）。上电扫描恢复写指针，
  * Flash 池写满后整池擦除重写（简化磨损均衡，docs 里讨论了完整方案）。
  ******************************************************************************
  */
#ifndef __BLACKBOX_H
#define __BLACKBOX_H

#include <stdint.h>

/* 事件类型 */
#define BB_EVT_FAULT_SET    0x01U   /* 故障置位（arg=故障号 1~3） */
#define BB_EVT_DTC_CLEAR    0x02U   /* 清码（arg=0xFF 全清） */
#define BB_EVT_SESSION      0x03U   /* 会话切换（arg=新会话号） */
#define BB_EVT_UNLOCKED     0x04U   /* 安全访问通过 */
#define BB_EVT_PARAM_WRITE  0x05U   /* 参数被诊断仪修改（arg=值高/低字节） */
#define BB_EVT_POWER_ON     0x06U   /* 上电 */

uint8_t  blackbox_mount(void);                  /* 上电挂载：扫描恢复写指针 */
void     blackbox_format(void);                 /* 清空全部日志（CLI: bbclear） */
void     blackbox_log(uint8_t type, uint8_t arg_a, uint8_t arg_b);
void     blackbox_dump(void);                   /* 打印全部记录（CLI 用） */
uint32_t blackbox_count(void);
const char *blackbox_evt_name(uint8_t type);

#endif /* __BLACKBOX_H */
