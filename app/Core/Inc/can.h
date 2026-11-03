/**
  ******************************************************************************
  * @file    can.h
  * @brief   bxCAN 驱动层：初始化 / 中断式收发 / 环形缓冲 / 上电自检
  ******************************************************************************
  * 本工程 CAN ID 分配（与 docs/CAN_UDS_PROTOCOL.md 保持一致）：
  *   0x310  数据广播帧：车速/转速/电压/温度，100ms 周期（模拟整车信号）
  *   0x7DF  UDS 功能寻址请求（广播给总线上所有 ECU）
  *   0x7E0  UDS 物理寻址请求（诊断仪 -> 本 ECU）
  *   0x7E8  UDS 响应（本 ECU -> 诊断仪）
  * （0x7E0/0x7E8 是 ISO 15765-4 规定的 OBD 诊断标准地址）
  ******************************************************************************
  */
#ifndef __CAN_H
#define __CAN_H

#include "stm32f1xx_hal.h"

/* ---------------- 返回码 ---------------- */
#define CAN_OK          0U
#define CAN_ERR_INIT    1U
#define CAN_ERR_SEND    2U
#define CAN_ERR_RECV    3U

/* ---------------- 协议 ID 表 ---------------- */
#define CAN_ID_VEHICLE_DATA   0x310U   /* 数据广播帧 */
#define CAN_ID_DIAG_FUNC      0x7DFU   /* UDS 功能寻址 */
#define CAN_ID_DIAG_REQ       0x7E0U   /* UDS 物理寻址请求 */
#define CAN_ID_DIAG_RESP      0x7E8U   /* UDS 响应 */

/* ---------------- 接收环形缓冲 ----------------
 * CAN_RX0 中断把新帧压进来，主循环的各个任务再取走。
 * 中断里只做"入队"，解析留给任务级 -> ISR 时间短，不丢帧。 */
#define CAN_RX_RING_SIZE  32U

typedef struct
{
    uint32_t id;                    /* 标准 ID（11 位） */
    uint8_t  len;                   /* DLC 0~8 */
    uint8_t  data[8];               /* 数据域 */
} can_frame_t;

void        can_ring_reset(void);
uint8_t     can_ring_pop(can_frame_t *f);        /* 取一帧，空则返回 0 */
uint8_t     can_ring_peek_diag_req(can_frame_t *f);  /* 只取 0x7DF/0x7E0（UDS 服务端用） */
uint8_t     can_ring_peek_diag_resp(can_frame_t *f); /* 只取 0x7E8（UDS 客户端用） */
void        can_ring_drop_vehicle(void);         /* 丢弃 0x310 自环回声（ECU/DEMO 用） */
uint32_t    can_ring_count(void);                /* 当前积压帧数（调试用） */

/* ---------------- 驱动接口 ---------------- */
uint8_t     can_init(uint32_t mode);             /* CAN_MODE_NORMAL / CAN_MODE_LOOPBACK */
uint8_t     can_send(uint32_t id, const uint8_t *data, uint8_t len);
uint8_t     can_recv(uint32_t *id, uint8_t *data, uint8_t *len, uint32_t timeout_ms);
void        can_irq_enable(void);                /* 使能 FIFO0 接收中断 + NVIC */
const char *can_mode_name(void);                 /* 当前模式字符串（打印用） */
uint32_t    can_tx_count(void);
uint32_t    can_rx_count(void);

int         can_selftest(void);                  /* 上电自环自检（T1/T2） */

extern CAN_HandleTypeDef hcan;

#endif /* __CAN_H */
