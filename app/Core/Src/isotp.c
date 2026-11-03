/**
  ******************************************************************************
  * @file    isotp.c
  * @brief   ISO-TP 子集实现（SF / FF / CF）
  ******************************************************************************
  */
#include "isotp.h"
#include "can.h"
#include <string.h>

/* 帧间小延时：连续帧发太快，接收端中断+环形缓冲也能扛住，
 * 但留 2ms 更贴近真实 STmin，也更稳 */
#define CF_GAP_MS  2U

void isotp_rx_reset(isotp_rx_t *ctx)
{
    ctx->total      = 0U;
    ctx->got        = 0U;
    ctx->expect_seq = 1U;
    ctx->active     = 0U;
}

/**
  * @brief  发送一条报文（<=7B 用单帧；更长用 FF + N 个 CF）
  */
uint8_t isotp_send(uint32_t can_id, const uint8_t *msg, uint16_t len)
{
    uint8_t frame[8];
    uint16_t off;
    uint8_t seq = 1U;

    if ((len == 0U) || (len > ISOTP_MAX_LEN)) return 1U;

    if (len <= 7U)                              /* --- 单帧 --- */
    {
        frame[0] = (uint8_t)(0x00U | len);      /* 高 nibble=0, 低 nibble=长度 */
        memcpy(&frame[1], msg, len);
        /* 剩余字节填 0x55（ISO 规定填充值任意，0x55 便于示波器辨认） */
        memset(&frame[1 + len], 0x55U, (size_t)(7U - len));
        return can_send(can_id, frame, 8);
    }

    /* --- 首帧：0x1 + 长度高 4 位 | 长度低 8 位 + 前 6 字节 --- */
    frame[0] = (uint8_t)(0x10U | (len >> 8));
    frame[1] = (uint8_t)(len & 0xFFU);
    memcpy(&frame[2], msg, 6U);
    if (can_send(can_id, frame, 8) != CAN_OK) return 1U;

    /* --- 连续帧 --- */
    off = 6U;
    while (off < len)
    {
        uint8_t n = (uint8_t)(len - off);
        if (n > 7U) n = 7U;

        frame[0] = (uint8_t)(0x20U | (seq & 0x0FU));
        memcpy(&frame[1], &msg[off], n);
        if (n < 7U) memset(&frame[1 + n], 0x55U, (size_t)(7U - n));

        if (can_send(can_id, frame, 8) != CAN_OK) return 1U;

        seq = (uint8_t)((seq + 1U) & 0x0FU);
        off = (uint16_t)(off + n);
        HAL_Delay(CF_GAP_MS);
    }
    return 0U;
}

/**
  * @brief  喂一帧给接收状态机
  * @retval >0 完整报文长度（buf 里已拼好）；0 还没收齐；<0 协议错误
  */
int16_t isotp_rx_feed(isotp_rx_t *ctx, const uint8_t *data, uint8_t len)
{
    uint8_t pci = (uint8_t)(data[0] & 0xF0U);

    switch (pci)
    {
    case 0x00U:                                 /* --- 单帧 --- */
    {
        uint8_t n = (uint8_t)(data[0] & 0x0FU);
        if (n > 7U || n == 0U) return -1;
        isotp_rx_reset(ctx);
        memcpy(ctx->buf, &data[1], n);
        return (int16_t)n;
    }

    case 0x10U:                                 /* --- 首帧：开始一条多帧报文 --- */
    {
        uint16_t total = (uint16_t)(((uint16_t)(data[0] & 0x0FU) << 8) | data[1]);
        if (total > ISOTP_MAX_LEN) return -2;
        isotp_rx_reset(ctx);
        ctx->active = 1U;
        ctx->total  = total;
        memcpy(ctx->buf, &data[2], 6U);
        ctx->got = 6U;
        return 0;
    }

    case 0x20U:                                 /* --- 连续帧 --- */
    {
        uint8_t seq = (uint8_t)(data[0] & 0x0FU);
        uint16_t remain;
        uint8_t  n;

        if (!ctx->active) return -3;            /* 没有进行中的 FF，孤帧丢弃 */
        if (seq != ctx->expect_seq) return -4;  /* 序号不连续：丢帧/乱序 */
        ctx->expect_seq = (uint8_t)((seq + 1U) & 0x0FU);

        remain = (uint16_t)(ctx->total - ctx->got);
        n = (uint8_t)((remain < 7U) ? remain : 7U);
        memcpy(&ctx->buf[ctx->got], &data[1], n);
        ctx->got = (uint16_t)(ctx->got + n);

        if (ctx->got >= ctx->total)             /* 拼齐了 */
        {
            ctx->active = 0U;
            return (int16_t)ctx->total;
        }
        return 0;
    }

    default:                                    /* 0x30 流控帧：本项目不用，忽略 */
        return 0;
    }
}
