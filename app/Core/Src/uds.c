/**
  ******************************************************************************
  * @file    uds.c
  * @brief   UDS 诊断服务实现（服务端 + 客户端）
  ******************************************************************************
  * 角色分工：
  *   服务端（ECU）：监听 0x7DF / 0x7E0 -> isotp 重组 -> 分发服务 -> 应答 0x7E8
  *   客户端（诊断仪）：uds_client_* 组请求发到 0x7E0，收 0x7E8 解析打印
  * 单板 DEMO 模式下两个栈同跑，靠 CAN 自环收发，形成完整的"一次真实诊断"
  ******************************************************************************
  */
#include "uds.h"
#include "isotp.h"
#include "can.h"
#include "vehicle.h"
#include "blackbox.h"
#include <stdio.h>
#include <string.h>

/* ---------------- ECU 内部状态 ---------------- */
static uint8_t  s_session;          /* 当前会话 */
static uint8_t  s_unlocked;         /* 0x27 解锁标志（写 DID 的前提） */
static uint16_t s_pending_seed;     /* 已发出未验证的种子 */
static uint32_t s_req_cnt, s_resp_cnt, s_nrc_cnt;
static isotp_rx_t s_srv_rx;         /* 服务端重组上下文 */

/* ---------------- DTC 表 ----------------
 * 3 字节 DTC + 状态字节。故障由 vehicle_inject_fault 置位，
 * 0x14 清码时全部清掉，同时写黑匣子日志。 */
typedef struct { uint8_t dtc[3]; uint8_t status; } dtc_entry_t;

static const dtc_entry_t s_dtc_def[3] = {
    { {0xC1, 0x00, 0x87}, 0x09 },   /* battery voltage high */
    { {0xC2, 0x01, 0x59}, 0x09 },   /* engine overspeed */
    { {0xC3, 0x01, 0x61}, 0x09 }    /* wheel speed implausible */
};
static dtc_entry_t s_dtc_tab[3];
#define DTC_STATUS_ACTIVE 0x09U     /* bit0 testFailed | bit3 confirmedDTC */

/* VIN（0xF190 响应体） */
static const char s_vin[17] = "LSGKB54A1NS123456";

void uds_server_init(void)
{
    memcpy(s_dtc_tab, s_dtc_def, sizeof(s_dtc_tab));
    s_session  = UDS_SESSION_DEFAULT;
    s_unlocked = 0U;
    isotp_rx_reset(&s_srv_rx);
}

/* 把当前注入掩码映射到 DTC 表：激活=0x09，未激活=清掉 */
static void dtc_sync_from_vehicle(void)
{
    uint8_t mask = vehicle_fault_active_mask();
    uint8_t i;

    for (i = 0; i < 3U; i++)
    {
        s_dtc_tab[i].status = (mask & (1U << i)) ? DTC_STATUS_ACTIVE : 0x00U;
    }
}

static uint8_t dtc_active_count(void)
{
    uint8_t i, n = 0;
    for (i = 0; i < 3U; i++)
    {
        if (s_dtc_tab[i].status & 0x01U) n++;
    }
    return n;
}

/* ---------------- 服务端：发响应 ---------------- */
static void srv_respond(const uint8_t *msg, uint16_t len)
{
    if (isotp_send(CAN_ID_DIAG_RESP, msg, len) == 0U) s_resp_cnt++;
}

static void srv_neg_respond(uint8_t sid, uint8_t nrc)
{
    uint8_t m[3];
    m[0] = 0x7FU;
    m[1] = sid;
    m[2] = nrc;
    s_nrc_cnt++;
    srv_respond(m, 3);
}

/* ---------------- 服务分发 ---------------- */
static void srv_dispatch(const uint8_t *req, uint16_t len)
{
    uint8_t sid = req[0];

    if (len < 1U) return;
    s_req_cnt++;

    switch (sid)
    {
    case 0x10U:                                     /* --- 会话控制 --- */
    {
        if (len != 2U) { srv_neg_respond(sid, UDS_NRC_INCORRECT_MSG_LEN); break; }
        if ((req[1] == UDS_SESSION_DEFAULT) || (req[1] == UDS_SESSION_EXTENDED))
        {
            s_session  = req[1];
            s_unlocked = 0U;                        /* 换会话必须重新解锁 */
            blackbox_log(BB_EVT_SESSION, req[1], 0);
            {
                uint8_t m[6] = { 0x50U, req[1], 0x00U, 0x19U, 0x01U, 0xF4U };
                srv_respond(m, 6);                  /* 50+sub+P2(25ms)+P2*10(5000ms) */
            }
        }
        else
        {
            srv_neg_respond(sid, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
        }
        break;
    }

    case 0x14U:                                     /* --- 清 DTC --- */
    {
        uint8_t m[1] = { 0x54U };
        if (len != 4U) { srv_neg_respond(sid, UDS_NRC_INCORRECT_MSG_LEN); break; }
        memset(s_dtc_tab, 0, sizeof(s_dtc_tab));    /* 教学版：直接全清 */
        vehicle_inject_fault(0);
        blackbox_log(BB_EVT_DTC_CLEAR, 0xFFU, 0);
        srv_respond(m, 1);
        break;
    }

    case 0x19U:                                     /* --- 读 DTC --- */
    {
        dtc_sync_from_vehicle();
        if ((len != 3U) || (req[1] != 0x01U && req[1] != 0x02U))
        {
            srv_neg_respond(sid, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
            break;
        }
        if (req[1] == 0x01U)                        /* 子功能 01：报告 DTC 数量 */
        {
            /* 59 01 FF <count> <available=09> */
            uint8_t m[5] = { 0x59U, 0x01U, 0xFFU, dtc_active_count(), 0x09U };
            srv_respond(m, 5);
        }
        else                                        /* 子功能 02：报告 DTC 列表 */
        {
            uint8_t m[4U * 4U + 3U];
            uint8_t i, n = dtc_active_count(), k = 0;
            m[k++] = 0x59U; m[k++] = 0x02U; m[k++] = 0x09U; m[k++] = n;
            for (i = 0; i < 3U; i++)
            {
                if (s_dtc_tab[i].status & 0x01U)
                {
                    m[k++] = s_dtc_tab[i].dtc[0];
                    m[k++] = s_dtc_tab[i].dtc[1];
                    m[k++] = s_dtc_tab[i].dtc[2];
                    m[k++] = s_dtc_tab[i].status;
                }
            }
            srv_respond(m, k);                      /* >8B，走 ISO-TP 多帧 */
        }
        break;
    }

    case 0x22U:                                     /* --- 读 DID --- */
    {
        if (len != 3U) { srv_neg_respond(sid, UDS_NRC_INCORRECT_MSG_LEN); break; }
        {
            uint16_t did = (uint16_t)((req[1] << 8) | req[2]);
            uint8_t  m[24];
            vehicle_data_t *v = vehicle_get();

            m[0] = 0x62U; m[1] = req[1]; m[2] = req[2];
            switch (did)
            {
            case UDS_DID_ENGINE_RPM:
                m[3] = (uint8_t)(v->rpm >> 8);  m[4] = (uint8_t)v->rpm;
                srv_respond(m, 5);
                break;
            case UDS_DID_VEHICLE_SPEED:
                m[3] = (uint8_t)(v->speed_d10 >> 8); m[4] = (uint8_t)v->speed_d10;
                srv_respond(m, 5);
                break;
            case UDS_DID_BATT_VOLT:
                m[3] = (uint8_t)(v->batt_mv10 >> 8); m[4] = (uint8_t)v->batt_mv10;
                srv_respond(m, 5);
                break;
            case UDS_DID_VIN:
                memcpy(&m[3], s_vin, 17U);
                srv_respond(m, 20);
                break;
            case UDS_DID_BROADCAST_MS:
                m[3] = 0x00U; m[4] = 0x64U;     /* 当前 100ms */
                srv_respond(m, 5);
                break;
            default:
                srv_neg_respond(sid, UDS_NRC_REQUEST_OUT_OF_RANGE);
                break;
            }
        }
        break;
    }

    case 0x2EU:                                     /* --- 写 DID（安全演示） --- */
    {
        uint16_t did;
        if (len != 5U) { srv_neg_respond(sid, UDS_NRC_INCORRECT_MSG_LEN); break; }
        did = (uint16_t)((req[1] << 8) | req[2]);
        if (did != UDS_DID_BROADCAST_MS)
        {
            srv_neg_respond(sid, UDS_NRC_REQUEST_OUT_OF_RANGE);
            break;
        }
        if (s_session != UDS_SESSION_EXTENDED)
        {
            srv_neg_respond(sid, UDS_NRC_CONDITIONS_NOT_CORRECT);   /* 先进扩展会话 */
            break;
        }
        if (!s_unlocked)
        {
            srv_neg_respond(sid, UDS_NRC_SECURITY_DENIED);          /* 先过安全访问 */
            break;
        }
        {
            uint16_t ms = (uint16_t)((req[3] << 8) | req[4]);
            if ((ms < 20U) || (ms > 1000U))
            {
                srv_neg_respond(sid, UDS_NRC_REQUEST_OUT_OF_RANGE);
            }
            else
            {
                vehicle_set_broadcast(1U);
                blackbox_log(BB_EVT_PARAM_WRITE, (uint8_t)(ms >> 8), (uint8_t)ms);
                {
                    uint8_t m[3] = { 0x6EU, req[1], req[2] };
                    srv_respond(m, 3);
                }
            }
        }
        break;
    }

    case 0x27U:                                     /* --- 安全访问（教学简化算法） --- */
    {
        if (s_session != UDS_SESSION_EXTENDED)
        {
            srv_neg_respond(sid, UDS_NRC_CONDITIONS_NOT_CORRECT);
            break;
        }
        if (req[1] == 0x01U)                        /* 请求种子 */
        {
            uint8_t m[4];
            s_pending_seed = (uint16_t)(0x1234U + (HAL_GetTick() & 0xFFU));
            m[0] = 0x67U; m[1] = 0x01U;
            m[2] = (uint8_t)(s_pending_seed >> 8);
            m[3] = (uint8_t)s_pending_seed;
            srv_respond(m, 4);
        }
        else if (req[1] == 0x02U)                   /* 发送密钥 */
        {
            uint16_t key_expect = (uint16_t)(s_pending_seed ^ 0x5A5AU);
            uint16_t key_got   = (uint16_t)((req[2] << 8) | req[3]);
            if (key_got == key_expect)
            {
                uint8_t m[2] = { 0x67U, 0x02U };
                s_unlocked = 1U;
                blackbox_log(BB_EVT_UNLOCKED, 0, 0);
                srv_respond(m, 2);
            }
            else
            {
                srv_neg_respond(sid, UDS_NRC_INVALID_KEY);
            }
        }
        else
        {
            srv_neg_respond(sid, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
        }
        break;
    }

    default:
        srv_neg_respond(sid, UDS_NRC_SERVICE_NOT_SUPPORTED);
        break;
    }
}

void uds_server_task(void)
{
    can_frame_t f;

    while (can_ring_peek_diag_req(&f))              /* 只取 0x7DF/0x7E0 */
    {
        int16_t n = isotp_rx_feed(&s_srv_rx, f.data, f.len);
        if (n > 0) srv_dispatch(s_srv_rx.buf, (uint16_t)n);
        else if (n < 0) srv_neg_respond(0x10U, UDS_NRC_INCORRECT_MSG_LEN);
    }
}

/* ---------------- 客户端（诊断仪） ---------------- */
void uds_client_send_raw(const uint8_t *msg, uint16_t len)
{
    (void)isotp_send(CAN_ID_DIAG_REQ, msg, len);
}

void uds_client_session(uint8_t sub)
{
    uint8_t m[2] = { 0x10U, sub };
    uds_client_send_raw(m, 2);
}

void uds_client_clear_dtc(void)
{
    uint8_t m[4] = { 0x14U, 0xFFU, 0xFFU, 0xFFU };  /* 0xFFFFFF = 所有组 */
    uds_client_send_raw(m, 4);
}

void uds_client_read_dtc_list(void)
{
    uint8_t m[3] = { 0x19U, 0x02U, 0x09U };
    uds_client_send_raw(m, 3);
}

void uds_client_read_did(uint16_t did)
{
    uint8_t m[3] = { 0x22U, (uint8_t)(did >> 8), (uint8_t)did };
    uds_client_send_raw(m, 3);
}

void uds_client_write_did(uint16_t did, uint16_t value)
{
    uint8_t m[5];
    m[0] = 0x2EU; m[1] = (uint8_t)(did >> 8); m[2] = (uint8_t)did;
    m[3] = (uint8_t)(value >> 8); m[4] = (uint8_t)value;
    uds_client_send_raw(m, 5);
}

void uds_client_security_access(void)
{
    uint8_t m[2] = { 0x27U, 0x01U };
    uds_client_send_raw(m, 2);                      /* 先请求种子 */
    /* 真实诊断仪应等种子回来再算 key；教学版由 CLI 分两步走 */
}

static void client_print_response(const uint8_t *msg, uint16_t len)
{
    uint16_t i;

    printf("  <- UDS resp (%uB):", len);
    for (i = 0; i < len; i++) printf(" %02X", msg[i]);
    printf("\r\n");

    if (msg[0] == 0x7FU)
    {
        printf("  == NEGATIVE (sid=0x%02X nrc=0x%02X)\r\n", msg[1], msg[2]);
    }
    else if (msg[0] == 0x62U)                       /* 读 DID 响应：顺手解析 */
    {
        uint16_t did = (uint16_t)((msg[1] << 8) | msg[2]);
        switch (did)
        {
        case UDS_DID_ENGINE_RPM:
            printf("  == engine rpm = %u\r\n", ((unsigned)msg[3] << 8) | msg[4]);
            break;
        case UDS_DID_VEHICLE_SPEED:
            printf("  == vehicle speed = %u.%u km/h\r\n",
                   (((unsigned)msg[3] << 8) | msg[4]) / 10U,
                   (((unsigned)msg[3] << 8) | msg[4]) % 10U);
            break;
        case UDS_DID_BATT_VOLT:
            printf("  == battery = %u.%02u V\r\n",
                   (((unsigned)msg[3] << 8) | msg[4]) / 100U,
                   (((unsigned)msg[3] << 8) | msg[4]) % 100U);
            break;
        case UDS_DID_VIN:
            printf("  == VIN = %.*s\r\n", 17, (const char *)&msg[3]);
            break;
        default:
            break;
        }
    }
    else if ((msg[0] == 0x67U) && (msg[1] == 0x01U))    /* 种子：自动算 key 回 */
    {
        uint16_t seed = (uint16_t)((msg[2] << 8) | msg[3]);
        uint16_t key  = (uint16_t)(seed ^ 0x5A5AU);
        uint8_t m[4];
        printf("  == seed=0x%04X, key=seed^0x5A5A=0x%04X, sending...\r\n", seed, key);
        m[0] = 0x27U; m[1] = 0x02U;
        m[2] = (uint8_t)(key >> 8); m[3] = (uint8_t)key;
        uds_client_send_raw(m, 4);
    }
    else if ((msg[0] == 0x59U) && (msg[1] == 0x02U))    /* DTC 列表：解析打印 */
    {
        uint8_t n = msg[3];
        uint8_t i;
        printf("  == %u DTC(s):\r\n", n);
        for (i = 0; i < n; i++)
        {
            uint16_t base = (uint16_t)(4U + i * 4U);
            printf("     DTC %02X%02X%02X status=%02X%s\r\n",
                   msg[base], msg[base + 1U], msg[base + 2U], msg[base + 3U],
                   (msg[base + 3U] & 0x01U) ? " (active)" : "");
        }
    }
}

void uds_client_task(void)
{
    can_frame_t f;
    static isotp_rx_t cli_rx;                       /* 客户端自己的重组上下文 */

    while (can_ring_peek_diag_resp(&f))             /* 只取 0x7E8 */
    {
        int16_t n = isotp_rx_feed(&cli_rx, f.data, f.len);
        if (n > 0) client_print_response(cli_rx.buf, (uint16_t)n);
    }
}

uint32_t uds_req_count(void)  { return s_req_cnt; }
uint32_t uds_resp_count(void) { return s_resp_cnt; }
uint32_t uds_nrc_count(void)  { return s_nrc_cnt; }
