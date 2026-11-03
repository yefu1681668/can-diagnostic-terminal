/**
  ******************************************************************************
  * @file    can.c
  * @brief   bxCAN 驱动：500kbps + 接收中断环形缓冲 + 自环自检
  ******************************************************************************
  * 波特率推导（C8T6 主频 72MHz，APB1 = 36MHz）：
  *   一位 = 1 同步段 + TS1(15) + TS2(2) = 18 时间片
  *   36MHz / 4(分频) / 18 = 500kbps，采样点 (1+15)/18 = 88.9%
  *
  * 自环模式（LBKM）：控制器内部把 TX 接回 RX，单板即可验证收发/过滤/分帧，
  * 也是 ROLE_DEMO 单板演示的基础——本机发的 UDS 请求会回到本机被 ECU 栈处理。
  ******************************************************************************
  */
#include "can.h"
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan;

/* ---------------- 位时序参数 ---------------- */
#define CAN_PRESCALER  4U
#define CAN_TS1        CAN_BS1_15TQ
#define CAN_TS2        CAN_BS2_2TQ
#define CAN_SJW        CAN_SJW_1TQ

/* ---------------- 运行状态 ---------------- */
static uint32_t s_mode;                     /* 记录当前模式（打印用） */
static uint32_t s_tx_cnt;                   /* 已成功发送帧数（canstats 用） */
static uint32_t s_rx_cnt;                   /* 中断收到总帧数 */

/* ---------------- 接收环形缓冲 ---------------- */
static can_frame_t s_ring[CAN_RX_RING_SIZE];
static volatile uint16_t s_head;            /* 写指针（ISR 用） */
static volatile uint16_t s_tail;            /* 读指针（任务用） */

void can_ring_reset(void)
{
    s_head = 0U;
    s_tail = 0U;
}

uint8_t can_ring_pop(can_frame_t *f)
{
    if (s_tail == s_head) return 0U;        /* 空队列 */
    *f = s_ring[s_tail];
    s_tail = (uint16_t)((s_tail + 1U) % CAN_RX_RING_SIZE);
    return 1U;
}

uint32_t can_ring_count(void)
{
    return (uint32_t)((s_head + CAN_RX_RING_SIZE - s_tail) % CAN_RX_RING_SIZE);
}

/* 内部：从环形缓冲里按谓词摘除一帧（边扫边判，找到即删） */
static uint8_t ring_remove_if(can_frame_t *f, uint8_t (*match)(const can_frame_t *))
{
    uint16_t i;
    uint16_t n = (uint16_t)((s_head + CAN_RX_RING_SIZE - s_tail) % CAN_RX_RING_SIZE);

    for (i = 0; i < n; i++)
    {
        uint16_t idx = (uint16_t)((s_tail + i) % CAN_RX_RING_SIZE);
        if (match(&s_ring[idx]))
        {
            *f = s_ring[idx];
            /* 后面的元素整体前移一格（缓冲小，搬移开销可忽略） */
            while (i > 0U)
            {
                uint16_t cur  = (uint16_t)((s_tail + i) % CAN_RX_RING_SIZE);
                uint16_t prev = (uint16_t)((s_tail + i - 1U) % CAN_RX_RING_SIZE);
                s_ring[prev] = s_ring[cur];
                i--;
            }
            s_tail = (uint16_t)((s_tail + 1U) % CAN_RX_RING_SIZE);
            return 1U;
        }
    }
    return 0U;
}

static uint8_t match_diag_req(const can_frame_t *f)
{
    return (uint8_t)((f->id == CAN_ID_DIAG_FUNC) || (f->id == CAN_ID_DIAG_REQ));
}

static uint8_t match_diag_resp(const can_frame_t *f)
{
    return (uint8_t)(f->id == CAN_ID_DIAG_RESP);
}

/* UDS 服务端用：只取诊断请求帧（0x7DF / 0x7E0），广播数据帧留给别的任务 */
uint8_t can_ring_peek_diag_req(can_frame_t *f)  { return ring_remove_if(f, match_diag_req); }

/* UDS 客户端用：只取诊断响应帧（0x7E8） */
uint8_t can_ring_peek_diag_resp(can_frame_t *f) { return ring_remove_if(f, match_diag_resp); }

/* ECU/DEMO 模式专用：0x310 广播帧是自己发的自环回声，本端不消费，直接丢弃。
 * 不丢的话环形缓冲会被自环数据塞满，诊断帧反而进不来。 */
void can_ring_drop_vehicle(void)
{
    /* 原地压缩：把非 0x310 的帧往前搬，0x310 被挤掉 */
    uint16_t rd = s_tail, wr = s_tail;
    while (rd != s_head)
    {
        can_frame_t cur = s_ring[rd];
        if (cur.id != CAN_ID_VEHICLE_DATA)
        {
            s_ring[wr] = cur;
            wr = (uint16_t)((wr + 1U) % CAN_RX_RING_SIZE);
        }
        rd = (uint16_t)((rd + 1U) % CAN_RX_RING_SIZE);
    }
    s_head = wr;
}

/* CAN_RX0 中断服务入口（stm32f1xx_it.c 里调用 HAL_CAN_IRQHandler，
 * HAL 再回调本函数：只做一件事——把硬件 FIFO 里的帧搬进软件环形缓冲） */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
    CAN_RxHeaderTypeDef rxh;
    can_frame_t f;
    uint16_t next;

    if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &rxh, f.data) != HAL_OK)
    {
        return;
    }
    f.id  = rxh.StdId;
    f.len = (uint8_t)rxh.DLC;
    s_rx_cnt++;

    next = (uint16_t)((s_head + 1U) % CAN_RX_RING_SIZE);
    if (next == s_tail) return;             /* 满了丢最新一帧（demo 流量小，不会发生） */
    s_ring[s_head] = f;
    s_head = next;
}

/**
  * @brief  初始化 CAN1（可重复调用：切换模式时先 DeInit）
  */
uint8_t can_init(uint32_t mode)
{
    CAN_FilterTypeDef f = {0};

    if (hcan.State != HAL_CAN_STATE_RESET)
    {
        HAL_CAN_Stop(&hcan);
        HAL_CAN_DeInit(&hcan);
    }

    hcan.Instance                  = CAN1;
    hcan.Init.Prescaler            = CAN_PRESCALER;
    hcan.Init.Mode                 = mode;
    hcan.Init.SyncJumpWidth        = CAN_SJW;
    hcan.Init.TimeSeg1             = CAN_TS1;
    hcan.Init.TimeSeg2             = CAN_TS2;
    hcan.Init.TimeTriggeredMode    = DISABLE;
    hcan.Init.AutoBusOff           = DISABLE;
    hcan.Init.AutoWakeUp           = DISABLE;
    hcan.Init.AutoRetransmission   = ENABLE;    /* 仲裁丢失/错误自动重发 */
    hcan.Init.ReceiveFifoLocked    = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        return CAN_ERR_INIT;
    }

    /* 过滤器 0：32 位掩码、掩码全 0 = 全收。
     * 教学项目流量小，全收 + 软件按 ID 分发最直观；量产应按协议表收紧。 */
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK)
    {
        return CAN_ERR_INIT;
    }

    can_ring_reset();

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        return CAN_ERR_INIT;
    }

    s_mode = mode;
    return CAN_OK;
}

void can_irq_enable(void)
{
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    /* 注意：中容量 F103 的 CAN1 RX0 与 USB 共用一根中断线，
     * 在 CMSIS 头文件里叫 USB_LP_CAN1_RX0_IRQn（不是 CAN_RX0_IRQn） */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

uint8_t can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef hdr;
    uint32_t mailbox = 0;
    uint32_t t0;

    if (len > 8U) len = 8U;

    hdr.StdId              = id;
    hdr.ExtId              = 0;
    hdr.IDE                = CAN_ID_STD;
    hdr.RTR                = CAN_RTR_DATA;
    hdr.DLC                = len;
    hdr.TransmitGlobalTime = DISABLE;

    /* 等 3 个硬件邮箱有空闲；50ms 等不到说明总线异常（如单节点无 ACK） */
    t0 = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
    {
        if (HAL_GetTick() - t0 > 50U) return CAN_ERR_SEND;
    }

    if (HAL_CAN_AddTxMessage(&hcan, &hdr, (uint8_t *)data, &mailbox) != HAL_OK)
    {
        return CAN_ERR_SEND;
    }

    t0 = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan, mailbox) != 0U)
    {
        if (HAL_GetTick() - t0 > 50U) return CAN_ERR_SEND;
    }

    s_tx_cnt++;
    return CAN_OK;
}

uint8_t can_recv(uint32_t *id, uint8_t *data, uint8_t *len, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0U)
    {
        if (HAL_GetTick() - t0 > timeout_ms) return CAN_ERR_RECV;
    }

    {
        CAN_RxHeaderTypeDef rxh;
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxh, data) != HAL_OK)
        {
            return CAN_ERR_RECV;
        }
        *id  = rxh.StdId;
        *len = (uint8_t)rxh.DLC;
    }
    return CAN_OK;
}

const char *can_mode_name(void)
{
    return (s_mode == CAN_MODE_LOOPBACK) ? "LOOPBACK" : "NORMAL";
}

uint32_t can_tx_count(void) { return s_tx_cnt; }
uint32_t can_rx_count(void) { return s_rx_cnt; }

/* ----------------------------------------------------------------------------
 * 上电自检（单板自环，两步）：
 *   T1  单帧 6B 回环          -> 验初始化 / 过滤器 / 收发通路
 *   T2  200B 分帧发送再重组    -> 验 ISO-TP 之前身的"拆包-重组"思路
 * 返回 0 = 通过；1/2/3/4 = 失败步骤编号
 * -------------------------------------------------------------------------- */
int can_selftest(void)
{
    uint8_t  tx[6], rx[8];
    uint8_t  payload[200], rebuild[200];
    uint32_t id;
    uint8_t  len;
    uint16_t off, n, i;

    printf("[CAN] init 500kbps (LOOPBACK) ... ");
    if (can_init(CAN_MODE_LOOPBACK) != CAN_OK)
    {
        printf("FAIL\r\n");
        return 1;
    }
    printf("OK\r\n");

    for (i = 0; i < 6U; i++) tx[i] = (uint8_t)(0xA0U + i);

    if (can_send(CAN_ID_VEHICLE_DATA, tx, 6) != CAN_OK) { printf("[CAN] T1 send FAIL\r\n"); return 2; }
    if (can_recv(&id, rx, &len, 200) != CAN_OK)          { printf("[CAN] T1 recv FAIL\r\n"); return 3; }
    if ((id != CAN_ID_VEHICLE_DATA) || (len != 6U) || (memcmp(tx, rx, 6) != 0))
    {
        printf("[CAN] T1 mismatch: id=0x%03lX len=%u\r\n", (unsigned long)id, len);
        return 4;
    }
    printf("[CAN] T1 pass: 6B frame loopback OK\r\n");

    for (i = 0; i < 200U; i++) payload[i] = (uint8_t)(i * 7U + 3U);

    off = 0;
    while (off < 200U)
    {
        n = (uint16_t)(200U - off);
        if (n > 8U) n = 8U;
        if (can_send(CAN_ID_VEHICLE_DATA, &payload[off], (uint8_t)n) != CAN_OK) { printf("[CAN] T2 send FAIL @%u\r\n", off); return 2; }
        if (can_recv(&id, rx, &len, 100) != CAN_OK)                             { printf("[CAN] T2 recv FAIL @%u\r\n", off); return 3; }
        memcpy(&rebuild[off], rx, n);
        off = (uint16_t)(off + n);
    }
    if (memcmp(payload, rebuild, 200U) != 0)
    {
        printf("[CAN] T2 data mismatch\r\n");
        return 4;
    }
    printf("[CAN] T2 pass: 200B / %u frames reassembled OK\r\n", (unsigned)((200U + 7U) / 8U));

    return 0;
}
