/**
  ******************************************************************************
  * @file    vehicle.h
  * @brief   模拟整车数据源：传感器信号 + 故障注入 + 100ms 周期广播
  ******************************************************************************
  * 真车上这些信号来自传感器/其他 ECU；教学项目里用一个"数字化的假车"：
  * 转速/车速按正弦规律变化，电压/温度缓变 —— 这样 CAN 总线上有真实感的
  * 持续流量，诊断仪端才有东西可读。
  ******************************************************************************
  */
#ifndef __VEHICLE_H
#define __VEHICLE_H

#include <stdint.h>

/* 0x310 广播帧布局（8 字节，大端）：
 * [0..1] 转速 rpm   [2..3] 车速 0.1km/h   [4] 电瓶电压 0.1V
 * [5]   冷却液温度(°C+50偏移)   [6] 状态位   [7] 计数器(丢帧检测用) */
typedef struct
{
    uint16_t rpm;               /* 发动机转速 */
    uint16_t speed_d10;         /* 车速 ×10 */
    uint16_t batt_mv10;         /* 电瓶电压 ×100（如 1380 = 13.80V） */
    int8_t   coolant_c;         /* 冷却液温度 */
    uint8_t  flags;             /* bit0=有激活故障 bit1=广播开 */
    uint8_t  seq;               /* 广播计数器，接收端靠它检测丢帧 */
} vehicle_data_t;

void         vehicle_init(void);
void         vehicle_task(void);               /* 主循环调用：100ms 广播 */
void         vehicle_set_broadcast(uint8_t on);
vehicle_data_t *vehicle_get(void);
const char  *vehicle_fault_name(uint8_t idx);   /* 故障名（打印用） */
void         vehicle_inject_fault(uint8_t idx); /* 1~3 注入, 0 清除全部 */
uint8_t      vehicle_fault_active_mask(void);

#endif /* __VEHICLE_H */
