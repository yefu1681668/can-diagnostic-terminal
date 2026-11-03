/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported constants --------------------------------------------------------*/
/* ---------------- 节点角色（改这里 + 重编译，一固件跑三种形态） ----------------
 *   ROLE_DEMO     单板演示：自环模式，ECU栈 + 诊断仪栈同跑（默认，一块板全体验）
 *   ROLE_ECU      双板真机：本板当 ECU（传感器模拟 + UDS 服务端），NORMAL 模式
 *   ROLE_GATEWAY  双板真机：本板当诊断终端（UDS 客户端 + 数据监听 + 黑匣子） */
#define ROLE_DEMO     0
#define ROLE_ECU      1
#define ROLE_GATEWAY  2

#ifndef NODE_ROLE
#define NODE_ROLE     ROLE_DEMO
#endif

#if (NODE_ROLE == ROLE_DEMO)
  #define NODE_ROLE_NAME   "DEMO"
  #define CAN_WORK_MODE    CAN_MODE_LOOPBACK
#elif (NODE_ROLE == ROLE_ECU)
  #define NODE_ROLE_NAME   "ECU"
  #define CAN_WORK_MODE    CAN_MODE_NORMAL
#else
  #define NODE_ROLE_NAME   "GATEWAY"
  #define CAN_WORK_MODE    CAN_MODE_NORMAL
#endif

/* 板载 LED */
#define LED_Pin       GPIO_PIN_13
#define LED_GPIO_Port GPIOC

/* W25Q 片选（PA4，软件控制） */
#define W25Q64_CS_Pin GPIO_PIN_4
#define W25Q64_CS_GPIO_Port GPIOA

/* Exported variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef  hspi1;

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
