/**
  ******************************************************************************
  * @file    isotp.h
  * @brief   ISO 15765-2 (ISO-TP) 子集：把 >8 字节的 UDS 报文拆进 CAN 帧
  ******************************************************************************
  * 帧类型（首字节高 4 位）：
  *   SF  0x0L  单帧，L<=7                直接装下，1 帧完事
  *   FF  0x1X XX  首帧，后 12 位=总长     第一帧带总长度，接收方据此开缓冲
  *   CF  0x2N  连续帧，N=序号(0~F 循环)   后续数据，序号防丢防乱
  *   FC  0x30 BS STmin  流控帧            本项目流量小，收发都不发 FC（简化）
  ******************************************************************************
  */
#ifndef __ISOTP_H
#define __ISOTP_H

#include <stdint.h>

#define ISOTP_MAX_LEN  64U      /* 本项目最大报文：DTC 列表响应（3*4+4=16B 足够，留余量） */

/* 发送：自动选择 SF 或 FF+CF，拆成多帧 can_send */
uint8_t isotp_send(uint32_t can_id, const uint8_t *msg, uint16_t len);

/* 接收状态机：喂进一帧 CAN 数据；拼齐一条完整报文返回其长度，未齐返回 0，出错返回负值 */
typedef struct
{
    uint8_t  buf[ISOTP_MAX_LEN];
    uint16_t total;             /* 期望总长 */
    uint16_t got;               /* 已收字节数 */
    uint8_t  expect_seq;        /* 下一个 CF 应带的序号 */
    uint8_t  active;            /* 正在拼一条多帧报文 */
} isotp_rx_t;

void    isotp_rx_reset(isotp_rx_t *ctx);
int16_t isotp_rx_feed(isotp_rx_t *ctx, const uint8_t *data, uint8_t len);

#endif /* __ISOTP_H */
