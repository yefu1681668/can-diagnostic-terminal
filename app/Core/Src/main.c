/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   车载 CAN 数据采集与故障诊断终端（独立程序，烧 0x08000000）
  ******************************************************************************
  * 三种运行形态（main.h 里 NODE_ROLE 切换）：
  *   DEMO    单板自环：ECU 栈 + 诊断栈同跑，串口 CLI 全流程演示
  *   ECU     双板真机：本板模拟整车 ECU（0x310 广播 + UDS 服务端 0x7E8）
  *   GATEWAY 双板真机：本板当诊断终端（UDS 客户端 0x7E0 + 黑匣子记录）
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "can.h"
#include "w25qxx.h"
#include "vehicle.h"
#include "uds.h"
#include "blackbox.h"
#include "cli.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
SPI_HandleTypeDef  hspi1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN 0 */
#if (NODE_ROLE == ROLE_GATEWAY)
/* ---------- GATEWAY 角色：监听 0x310 数据广播 ----------
 * 每秒汇总打印一次最新车速/转速；seq 不连续说明丢帧（总线质量指标）。
 * 故障标志位变化时写黑匣子 —— 这就是"数据采集 + 事件记录"的原型。 */
static void gateway_monitor_task(void)
{
    static uint32_t last_print = 0;
    static uint8_t  last_flags = 0xFFU;
    static uint8_t  last_seq   = 0U;
    static uint8_t  seq_init   = 0U;
    static vehicle_data_t v;        /* static：没有新帧时保持上次值 */
    static uint8_t  v_valid = 0U;   /* 收到第一帧前不打印，避免输出假数据 */
    can_frame_t f;

    while (can_ring_pop(&f))
    {
        if (f.id != CAN_ID_VEHICLE_DATA) continue;
        if (f.len < 8U) continue;

        v.rpm       = (uint16_t)((f.data[0] << 8) | f.data[1]);
        v.speed_d10 = (uint16_t)((f.data[2] << 8) | f.data[3]);
        v.batt_mv10 = (uint16_t)((f.data[4] << 8) | f.data[5]);
        v.flags     = f.data[6];
        v.seq       = f.data[7];
        v_valid     = 1U;

        if (seq_init && (v.seq != (uint8_t)(last_seq + 1U)))
        {
            printf("[GW] seq gap %u -> %u (frame lost!)\r\n", last_seq, v.seq);
        }
        last_seq = v.seq;
        seq_init = 1U;

        if (v.flags != last_flags)              /* 故障标志变化 = 事件 */
        {
            blackbox_log((v.flags & 0x01U) ? BB_EVT_FAULT_SET : BB_EVT_DTC_CLEAR,
                         0, v.flags);
            last_flags = v.flags;
        }
    }

    if (v_valid && ((HAL_GetTick() - last_print) >= 1000U))
    {
        last_print = HAL_GetTick();
        printf("[GW] rpm=%u speed=%u.%u batt=%u.%02uV flags=0x%02X\r\n",
               v.rpm, v.speed_d10 / 10U, v.speed_d10 % 10U,
               v.batt_mv10 / 100U, v.batt_mv10 % 100U, v.flags);
    }
}
#endif /* NODE_ROLE == ROLE_GATEWAY */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(100);                                   /* 等串口电平稳定 */

  /* ---------- 外部 Flash：黑匣子的存储介质 ---------- */
  printf("\r\n=== STM32 CAN Diagnostic Terminal (role=%s) ===\r\n", NODE_ROLE_NAME);
  W25Q_Init();                                      /* 先建 CS 空闲 + 等器件闲 */
  printf("[W25Q] JEDEC ID = 0x%06lX\r\n", (unsigned long)W25Q_ReadID());
  blackbox_mount();                                 /* 挂载事件日志（首次会格式化） */

  /* ---------- CAN：先自环自检，再切到工作模式 ---------- */
  {
      int st = can_selftest();
      if (st == 0)
      {
          printf("[CAN] self-test PASS\r\n");
      }
      else
      {
          printf("[CAN] self-test FAIL at step %d (continue in demo-safe mode)\r\n", st);
      }
  }

  /* 工作模式：DEMO=自环 / ECU、GATEWAY=真实总线 */
  can_init(CAN_WORK_MODE);
  can_irq_enable();                                 /* 之后所有收帧走中断环形缓冲 */
  printf("[CAN] %s mode, 500kbps, RX interrupt enabled\r\n", can_mode_name());

  /* ---------- 各业务栈 ---------- */
  vehicle_init();
  uds_server_init();
  cli_init();

  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  printf("[READY] type 'help' ...\r\ncmd> ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t led_last = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    cli_poll();                                     /* 串口命令行 */

#if (NODE_ROLE != ROLE_GATEWAY)
    can_ring_drop_vehicle();                        /* 清掉 0x310 自环回声，防塞满 */
    vehicle_task();                                 /* 100ms 传感数据广播（ECU/DEMO） */
    uds_server_task();                              /* UDS 服务端应答 */
#endif

#if (NODE_ROLE != ROLE_ECU)
    uds_client_task();                              /* UDS 客户端收响应 */
#endif

#if (NODE_ROLE == ROLE_GATEWAY)
    gateway_monitor_task();                         /* 0x310 数据监听 + 黑匣子 */
#endif

    /* 500ms 心跳：LED 翻转（PC13 低电平点亮） */
    if ((HAL_GetTick() - led_last) >= 500U)
    {
        led_last = HAL_GetTick();
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (HSE 8MHz -> PLL x9 = 72MHz)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Init: PC13 输出（LED）+ PA4 输出（W25Q64 片选）
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 时钟：GPIOC 给 LED，GPIOA 给 W25Q 片选（漏了 GPIOA 片选就永远拉不低 → ID 读 0） */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PC13 LED：初始高=灭 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PA4 W25Q64_CS：推挽输出，初始高（空闲释放，W25Q 在 CS 下降沿锁存命令） */
  HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = W25Q64_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W25Q64_CS_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief USART1 Init: 115200 8N1（PA9=TX, PA10=RX，接 CH340）
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Init: 主机模式，Mode0，8 分频（9MHz），软 NSS（PA4 片选）
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;        /* CPOL=0 */
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;            /* CPHA=0 -> Mode0，W25Q 要求 */
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* printf 重定向到 USART1 */
int fputc(int ch, FILE *f)
{
    (void)f;
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
