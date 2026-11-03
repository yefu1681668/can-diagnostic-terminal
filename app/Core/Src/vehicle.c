/**
  ******************************************************************************
  * @file    vehicle.c
  * @brief   模拟整车数据源实现
  ******************************************************************************
  * 数据都不是真的：转速/车速用正弦函数造出来，故障靠命令注入。
  * 但帧格式、广播周期、状态位是按真实车载网络的习惯设计的，
  * 将来换成真实传感器只需改这一个文件（这就是分层的好处）。
  ******************************************************************************
  */
#include "vehicle.h"
#include "can.h"
#include <string.h>

/* 故障定义：与 uds.c 里的 DTC 表一一对应 */
static vehicle_data_t s_v;
static uint8_t  s_broadcast = 1U;
static uint32_t s_last_tick;
static uint8_t  s_fault_mask;               /* bit0~2 = 故障 1~3 是否激活 */

/* 三个可注入的故障（名字与 DTC 含义对应，面试可讲） */
static const char *const s_fault_name[3] = {
    "battery voltage high",                 /* DTC 0xC10087 */
    "engine overspeed",                     /* DTC 0xC20159 */
    "wheel speed signal implausible"        /* DTC 0xC30161 */
};

void vehicle_init(void)
{
    memset(&s_v, 0, sizeof(s_v));
    s_v.rpm       = 800U;       /* 怠速 */
    s_v.speed_d10 = 0U;
    s_v.batt_mv10 = 1380U;      /* 13.8V，发电机充电电压 */
    s_v.coolant_c = 25;
    s_last_tick   = 0U;
}

vehicle_data_t *vehicle_get(void) { return &s_v; }

void vehicle_set_broadcast(uint8_t on) { s_broadcast = on ? 1U : 0U; }

uint8_t vehicle_fault_active_mask(void) { return s_fault_mask; }

const char *vehicle_fault_name(uint8_t idx)
{
    if (idx == 0U || idx > 3U) return "unknown";
    return s_fault_name[idx - 1U];
}

void vehicle_inject_fault(uint8_t idx)
{
    if (idx == 0U)
    {
        s_fault_mask = 0U;                          /* 清除全部 */
    }
    else if (idx <= 3U)
    {
        s_fault_mask |= (uint8_t)(1U << (idx - 1U));/* 置位对应故障 */
    }
    s_v.flags = (uint8_t)((s_fault_mask != 0U) ? 0x01U : 0x00U);
}

/**
  * @brief  每 100ms 更新一次模拟数据并广播一帧
  *         rpm = 800 + 2200*|sin(t/5s)|，speed 与 rpm 联动，造出"像在开车"的曲线
  */
void vehicle_task(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  frame[8];
    uint32_t t;

    if ((now - s_last_tick) < 100U) return;     /* 未到 100ms 周期 */
    s_last_tick = now;

    t = now / 1000U;                            /* 秒 */
    {
        /* 定点正弦：32 级查表近似，避免引入 math 库 */
        static const uint16_t sine32[33] = {
            0,98,196,290,381,466,545,616,679,732,776,809,831,842,842,831,
            809,776,732,679,616,545,466,381,290,196,98,0,0,0,0,0,0};
        uint16_t s = sine32[(t % 8U) * 4U];     /* 8 秒一个周期，取 32 点表 */
        s_v.rpm = (uint16_t)(800U + (uint32_t)s * 3U);          /* 800 ~ 3326 rpm */
        s_v.speed_d10 = (uint16_t)(s_v.rpm / 5U);               /* 车速与转速近似联动 */
        if ((t / 8U) & 1U)                                      /* 每 8s 电压波动一次 */
        {
            s_v.batt_mv10 = 1380U;
        }
        else
        {
            s_v.batt_mv10 = 1350U;
        }
        s_v.coolant_c = (int8_t)(25 + (t % 60U) / 2U);          /* 缓慢升温到 ~55 */
    }

    s_v.flags = (uint8_t)(((s_fault_mask != 0U) ? 0x01U : 0x00U)
                          | ((s_broadcast != 0U) ? 0x02U : 0x00U));

    if (!s_broadcast) return;                   /* 广播关闭（demo 可切换） */

    /* 组帧并广播：大端序，字节含义见 vehicle.h 注释 */
    frame[0] = (uint8_t)(s_v.rpm >> 8);
    frame[1] = (uint8_t)(s_v.rpm & 0xFFU);
    frame[2] = (uint8_t)(s_v.speed_d10 >> 8);
    frame[3] = (uint8_t)(s_v.speed_d10 & 0xFFU);
    frame[4] = (uint8_t)(s_v.batt_mv10 >> 8);
    frame[5] = (uint8_t)(s_v.batt_mv10 & 0xFFU);
    frame[6] = s_v.flags;
    frame[7] = s_v.seq++;

    (void)can_send(CAN_ID_VEHICLE_DATA, frame, 8);
}
