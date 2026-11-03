#ifndef W25QXX_H
#define W25QXX_H

#include "main.h"   /* 为了拿到 hspi1 句柄 和 W25Q64_CS 引脚宏 */

/* ============ W25Qxx 指令集（华邦 SPI NOR Flash 通用，W25Q16/32/64/128 通用）============ */
#define W25Q_CMD_WRITE_ENABLE   0x06U   /* 写使能：任何擦/写前必须先发 */
#define W25Q_CMD_WRITE_DISABLE  0x04U
#define W25Q_CMD_READ_ID        0x9FU   /* 读 JEDEC ID（厂牌+类型+容量） */
#define W25Q_CMD_READ_STATUS1   0x05U   /* 状态寄存器1，bit0=BUSY */
#define W25Q_CMD_READ_STATUS2   0x35U
#define W25Q_CMD_SECTOR_ERASE   0x20U   /* 扇区擦除（最小擦除单位 4KB） */
#define W25Q_CMD_PAGE_PROGRAM   0x02U   /* 页写（一页 256 字节） */
#define W25Q_CMD_READ_DATA      0x03U   /* 读数据（普通速度，无需哑元） */
#define W25Q_CMD_CHIP_ERASE     0xC7U

/* ============ 几何参数（全系列一致）============ */
#define W25Q_SECTOR_SIZE  4096U   /* 1 个扇区 = 4KB，擦除的最小粒度 */
#define W25Q_PAGE_SIZE    256U    /* 1 页 = 256 字节，写入的最小粒度 */

/* 状态寄存器1 的 BUSY 位（bit0）：1=器件忙（擦/写进行中），必须等它变 0 */
#define W25Q_STATUS1_BUSY 0x01U

/* ============ 对外函数（boot 主逻辑会调用）============ */
void     W25Q_Init(void);                       /* 上电初始化 CS 空闲 + 等闲 */
uint32_t W25Q_ReadID(void);                     /* 返回 24 位 ID（高8位=厂牌） */
void     W25Q_WriteEnable(void);                /* 写使能（擦/写前的门锁） */
void     W25Q_WaitBusy(void);                   /* 阻塞等待 BUSY 清零 */
void     W25Q_SectorErase(uint32_t addr);       /* 擦除 addr 所在的 4KB 扇区 */
void     W25Q_PageProgram(uint32_t addr,        /* 写一页，len<=256 且不能跨页 */
                          const uint8_t *data, uint16_t len);
void     W25Q_Read(uint32_t addr,              /* 任意长度读 */
                   uint8_t *buf, uint32_t len);

#endif /* W25QXX_H */
