#include "w25qxx.h"

/* hspi1 在 main.c 里定义为全局，这里用 extern 引用（CubeMX 惯例） */
extern SPI_HandleTypeDef hspi1;

/* ---------- CS 片选：软件控制 PA4，空闲必须高 ---------- */
static void W25Q_CS_Low(void)
{
    HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_RESET); /* 拉低=选中 */
}
static void W25Q_CS_High(void)
{
    HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_SET);   /* 拉高=释放 */
}

/* 发单字节命令（自带 CS 拉低/释放） */
static void W25Q_SendCmd(uint8_t cmd)
{
    W25Q_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    W25Q_CS_High();
}

/* ============ 上电初始化 ============ */
void W25Q_Init(void)
{
    W25Q_CS_High();     /* 空闲态 CS 必须高，W25Q 在 CS 下降沿锁存命令 */
    W25Q_WaitBusy();    /* 上电可能还在完成上一次操作，先等它闲 */
}

/* ============ 读 JEDEC ID（24位）============ */
uint32_t W25Q_ReadID(void)
{
    uint8_t cmd = W25Q_CMD_READ_ID;
    uint8_t id[3] = {0};
    W25Q_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);          /* 先发命令 */
    HAL_SPI_Receive(&hspi1, id, 3, 100);             /* 再收 3 字节 */
    W25Q_CS_High();
    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

/* ============ 写使能 ============ */
void W25Q_WriteEnable(void)
{
    W25Q_SendCmd(W25Q_CMD_WRITE_ENABLE);   /* 擦除/写入前必须发一次，芯片内部锁存 */
}

/* ============ 等忙 ============ */
void W25Q_WaitBusy(void)
{
    uint8_t cmd = W25Q_CMD_READ_STATUS1;
    uint8_t status = 0;
    W25Q_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);        /* 发读状态寄存器1命令 */
    do {
        HAL_SPI_Receive(&hspi1, &status, 1, 100);  /* 反复读，直到 BUSY 位=0 */
    } while (status & W25Q_STATUS1_BUSY);
    W25Q_CS_High();
}

/* ============ 扇区擦除（4KB）============ */
void W25Q_SectorErase(uint32_t addr)
{
    uint8_t tx[4];
    W25Q_WriteEnable();                  /* ① 擦除前必须先写使能 */
    tx[0] = W25Q_CMD_SECTOR_ERASE;
    tx[1] = (addr >> 16) & 0xFFU;        /* ② 24 位地址，大端（MSB 先发） */
    tx[2] = (addr >> 8)  & 0xFFU;
    tx[3] = (addr >> 0)  & 0xFFU;
    W25Q_CS_Low();
    HAL_SPI_Transmit(&hspi1, tx, 4, 100);
    W25Q_CS_High();
    W25Q_WaitBusy();                     /* ③ 擦一个扇区约 15~40ms，必须等完 */
}

/* ============ 页写（≤256字节，不跨页）============ */
void W25Q_PageProgram(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint8_t tx[4];
    if (len > W25Q_PAGE_SIZE) len = W25Q_PAGE_SIZE;   /* 一页最多 256 字节 */
    W25Q_WriteEnable();                  /* ① 写前必须先写使能 */
    tx[0] = W25Q_CMD_PAGE_PROGRAM;
    tx[1] = (addr >> 16) & 0xFFU;        /* ② 命令 + 24 位地址 */
    tx[2] = (addr >> 8)  & 0xFFU;
    tx[3] = (addr >> 0)  & 0xFFU;
    W25Q_CS_Low();
    HAL_SPI_Transmit(&hspi1, tx, 4, 100);            /* 先发命令+地址 */
    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 1000); /* 再发数据 */
    W25Q_CS_High();
    W25Q_WaitBusy();                     /* ③ 写一页约 0.5~3ms，必须等完 */
}

/* ============ 任意长度读 ============ */
void W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t tx[4];
    tx[0] = W25Q_CMD_READ_DATA;
    tx[1] = (addr >> 16) & 0xFFU;
    tx[2] = (addr >> 8)  & 0xFFU;
    tx[3] = (addr >> 0)  & 0xFFU;
    W25Q_CS_Low();
    HAL_SPI_Transmit(&hspi1, tx, 4, 100);   /* 先发命令+地址 */
    HAL_SPI_Receive(&hspi1, buf, len, 1000); /* 再收数据 */
    W25Q_CS_High();
}
