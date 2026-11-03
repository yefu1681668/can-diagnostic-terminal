/**
  ******************************************************************************
  * @file    blackbox.c
  * @brief   W25Q 环形事件日志实现
  ******************************************************************************
  * 存储布局（W25Q16 = 2MB，这里只占用前 64KB 演示）：
  *   扇区 0（4KB）：元数据 —— magic(4B) + 记录数(4B)
  *   扇区 1..3   ：记录池 —— 每条记录 16B，一扇区 256 条，共 768 条
  *
  * 记录格式（16B 定长，读出时 0xFF=空）：
  *   [0] 类型  [1..4] 时间戳 ms(小端)  [5..6] 参数  [7..15] 保留 0x00
  *
  * 写满 3 个扇区后整池擦除重来。真实产品应该做"先擦下一个扇区再写"的
  * 滚动擦除 + 磨损均衡，docs/BLACKBOX_DESIGN.md 里讨论了。
  ******************************************************************************
  */
#include "blackbox.h"
#include "w25qxx.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

#define BB_MAGIC        0x42424F58U     /* "BBOX" */
#define BB_META_ADDR    0x00000000U
#define BB_POOL_ADDR    0x00001000U     /* 扇区 1 起始 */
#define BB_POOL_SECTORS 3U
#define BB_REC_SIZE     16U
#define BB_POOL_CAP     (BB_POOL_SECTORS * (W25Q_SECTOR_SIZE / BB_REC_SIZE))  /* 768 */

/* 记录必须按 16B 紧凑字节布局【手工序列化】写入。
 * 绝对不要用 struct 直写 —— uint32_t 成员会引入 3 字节对齐填充，
 * 与 dump 的紧凑解析错位，时间戳/参数会变成乱码（真机踩过）。 */

static uint32_t s_count;            /* 累计记录数（含已被覆盖的） */
static uint8_t  s_mounted;

/**
  * @brief  格式化黑匣子：清元数据扇区 + 整个记录池，计数归零
  */
void blackbox_format(void)
{
    uint8_t  init[8];
    uint32_t i;

    init[0] = (uint8_t)(BB_MAGIC >> 24);
    init[1] = (uint8_t)(BB_MAGIC >> 16);
    init[2] = (uint8_t)(BB_MAGIC >> 8);
    init[3] = (uint8_t)BB_MAGIC;
    init[4] = init[5] = init[6] = init[7] = 0U;

    W25Q_WriteEnable();
    W25Q_SectorErase(BB_META_ADDR);
    W25Q_PageProgram(BB_META_ADDR, init, 8);

    for (i = 0U; i < BB_POOL_SECTORS; i++)
    {
        W25Q_WriteEnable();
        W25Q_SectorErase(BB_POOL_ADDR + i * W25Q_SECTOR_SIZE);
    }

    s_count = 0U;
}

/**
  * @brief  上电挂载：读元数据恢复计数；magic 不对（首次上电）则格式化
  */
uint8_t blackbox_mount(void)
{
    uint8_t hdr[8];

    W25Q_Init();

    W25Q_Read(BB_META_ADDR, hdr, 8);
    {
        uint32_t magic = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16)
                       | ((uint32_t)hdr[2] << 8)  |  hdr[3];
        s_count = ((uint32_t)hdr[4] << 24) | ((uint32_t)hdr[5] << 16)
                | ((uint32_t)hdr[6] << 8)  |  hdr[7];

        if (magic != BB_MAGIC)
        {
            blackbox_format();      /* 首次上电 / 版本不符：整块格式化 */
        }
    }
    s_mounted = 1U;
    blackbox_log(BB_EVT_POWER_ON, 0U, 0U);
    return 1U;
}

/**
  * @brief  追加一条事件（同步写：教学板流量低，直接写即可）
  */
void blackbox_log(uint8_t type, uint8_t arg_a, uint8_t arg_b)
{
    uint8_t  raw[BB_REC_SIZE];
    uint32_t ts;
    uint16_t arg;
    uint32_t slot;
    uint32_t addr;

    if (!s_mounted) return;

    ts  = HAL_GetTick();
    arg = (uint16_t)(((uint16_t)arg_a << 8) | arg_b);

    /* 手工填 16B：type(1) + ts(4,小端) + arg(2,高字节在前) + 保留(9) */
    memset(raw, 0, sizeof(raw));
    raw[0] = type;
    raw[1] = (uint8_t)(ts);
    raw[2] = (uint8_t)(ts >> 8);
    raw[3] = (uint8_t)(ts >> 16);
    raw[4] = (uint8_t)(ts >> 24);
    raw[5] = (uint8_t)(arg >> 8);
    raw[6] = (uint8_t)(arg);

    slot = s_count % BB_POOL_CAP;
    addr = BB_POOL_ADDR + slot * BB_REC_SIZE;

    /* 跨扇区第一笔：先擦这个扇区（整池已写过一轮的情况） */
    if ((slot % (W25Q_SECTOR_SIZE / BB_REC_SIZE)) == 0U)
    {
        W25Q_WriteEnable();
        W25Q_SectorErase(addr);
    }

    /* W25Q 页写一次最多 256B 且不能跨页；16B 记录对齐好了不会跨 */
    W25Q_WriteEnable();
    W25Q_PageProgram(addr, raw, BB_REC_SIZE);

    s_count++;

    /* 把新计数刷回元数据扇区：先擦后写 */
    {
        uint8_t hdr[8];
        hdr[0] = (uint8_t)(BB_MAGIC >> 24);
        hdr[1] = (uint8_t)(BB_MAGIC >> 16);
        hdr[2] = (uint8_t)(BB_MAGIC >> 8);
        hdr[3] = (uint8_t)BB_MAGIC;
        hdr[4] = (uint8_t)(s_count >> 24);
        hdr[5] = (uint8_t)(s_count >> 16);
        hdr[6] = (uint8_t)(s_count >> 8);
        hdr[7] = (uint8_t)s_count;
        W25Q_WriteEnable();
        W25Q_SectorErase(BB_META_ADDR);
        W25Q_PageProgram(BB_META_ADDR, hdr, 8);
    }
}

/**
  * @brief  按时间顺序打印记录（最早的在前；被覆盖的已不可恢复）
  */
void blackbox_dump(void)
{
    uint32_t i;
    uint32_t start = (s_count > BB_POOL_CAP) ? (s_count - BB_POOL_CAP) : 0U;

    printf("[BB] total logged=%lu, showing last %lu:\r\n",
           (unsigned long)s_count, (unsigned long)(s_count - start));

    for (i = start; i < s_count; i++)
    {
        uint8_t raw[BB_REC_SIZE];
        uint32_t addr = BB_POOL_ADDR + (i % BB_POOL_CAP) * BB_REC_SIZE;

        W25Q_Read(addr, raw, BB_REC_SIZE);
        if (raw[0] == 0xFFU) continue;          /* 空槽 */

        printf("  #%03lu %-12s t=%8lums arg=0x%02X%02X\r\n",
               (unsigned long)i, blackbox_evt_name(raw[0]),
               (unsigned long)((uint32_t)raw[1] | ((uint32_t)raw[2] << 8)
                             | ((uint32_t)raw[3] << 16) | ((uint32_t)raw[4] << 24)),
               raw[5], raw[6]);
    }
}

uint32_t blackbox_count(void) { return s_count; }

const char *blackbox_evt_name(uint8_t type)
{
    switch (type)
    {
    case BB_EVT_FAULT_SET:   return "FAULT_SET";
    case BB_EVT_DTC_CLEAR:   return "DTC_CLEAR";
    case BB_EVT_SESSION:     return "SESSION";
    case BB_EVT_UNLOCKED:    return "UNLOCKED";
    case BB_EVT_PARAM_WRITE: return "PARAM_WRITE";
    case BB_EVT_POWER_ON:    return "POWER_ON";
    default:                 return "UNKNOWN";
    }
}
