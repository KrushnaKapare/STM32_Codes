/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (merged - ultrasonic + CAN TX)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint32_t ic_val1 = 0;
volatile uint32_t ic_val2 = 0;
volatile uint8_t is_first_captured = 0; // 0 = not yet, 1 = first captured
volatile uint8_t capture_done = 0;

/* CAN TX */
CAN_TxHeaderTypeDef canTxHeader;
uint8_t canTxData[8];
uint32_t canTxMailbox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void DWT_Delay_Init(void);
static void delay_us(uint32_t us);

/* Minimal print function wrapper */
static void uart_printf(const char *fmt, ...);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();          // enable DWT cycle counter for microsecond delays

  /* Start input capture interrupt on TIM2 CH1 */
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Prepare CAN TX header (send 4 bytes float) */
  canTxHeader.DLC = 4;               // 4 bytes for float
  canTxHeader.IDE = CAN_ID_STD;
  canTxHeader.RTR = CAN_RTR_DATA;
  canTxHeader.StdId = 0x123;         // <--- change CAN ID as needed
  canTxHeader.TransmitGlobalTime = DISABLE;

  uart_printf("STM32F407VG Ultrasonic (TIM2 IC -> CAN1 & USART3)\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
      /* Trigger the sensor: 10us HIGH pulse on TRIG (PA1) */
      is_first_captured = 0;
      capture_done = 0;
      __HAL_TIM_SET_COUNTER(&htim2, 0); // reset counter

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
      delay_us(10); // 10 microsecond trigger
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

      /* Wait for capture to complete or timeout (50 ms) */
      uint32_t start = HAL_GetTick();
      while (!capture_done)
      {
          if ((HAL_GetTick() - start) > 50) // timeout 50ms
          {
              uart_printf("Timeout: no echo\r\n");
              break;
          }
      }

      if (capture_done)
      {
          uint32_t diff;
          /* TIM2 configured with prescaler such that tick = 1us (SystemCoreClock/84 -> 1MHz)
             Timer Period = 65535 -> 16-bit, handle overflow accordingly */
          if (ic_val2 >= ic_val1)
              diff = ic_val2 - ic_val1;
          else
              diff = (0xFFFF - ic_val1) + ic_val2 + 1; // handle 16-bit overflow

          /* Speed of sound: ~343 m/s = 0.0343 cm/us
             Distance (cm) = (time_us * 0.0343) / 2 = time_us * 0.01715
          */
          float distance_cm = (float)diff * 0.01715f;
          uart_printf("Pulse: %lu us, Distance: %.2f cm\r\n", (unsigned long)diff, distance_cm);

          /* ------ SEND VIA CAN ------ */
          memcpy(canTxData, &distance_cm, 4); // float -> 4 bytes, little-endian

          if (HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, canTxData, &canTxMailbox) != HAL_OK)
          {
              uart_printf("CAN TX Error\r\n");
          }
          else
          {
              /* Optionally wait until message leaves mailbox or check HAL status */
              uart_printf("CAN Sent: %.2f cm (ID: 0x%03lX)\r\n", distance_cm, (unsigned long)canTxHeader.StdId);
          }
      }

      HAL_Delay(200); // measure ~5 times / second
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  /* APB1CLKDivider = HCLK/2 (for TIM2 prescaler base assumptions) */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{
  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN CAN1_Init 2 */
  /* Configure CAN filter to accept all messages (change for production) */
  CAN_FilterTypeDef canFilter;
  canFilter.FilterActivation = ENABLE;
  canFilter.FilterBank = 0;
  canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilter.FilterIdHigh = 0x0000;
  canFilter.FilterIdLow = 0x0000;
  canFilter.FilterMaskIdHigh = 0x0000;
  canFilter.FilterMaskIdLow = 0x0000;
  canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilter.SlaveStartFilterBank = 14; // default for single CAN

  if (HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK)
  {
      Error_Handler();
  }

  /* Start CAN */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Enable RX interrupt (message pending) */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
      Error_Handler();
  }

  /* Enable CAN RX0 IRQ in NVIC (optional, HAL CAN IRQ handler used) */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE END CAN1_Init 2 */
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  /* Prescaler chosen to generate 1 MHz timer clock assuming SystemCoreClock = 84 MHz:
     Prescaler = 84 - 1 -> timer tick = 1 us */
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535; /* 16-bit top */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* Input capture channel config */
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable TIM2 IRQ in NVIC for HAL TIM handling */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level : PA1 (TRIG) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level : LEDs PD12..PD15 */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 (TRIG) */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 (LEDs) */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 (TIM2 CH1) as AF for TIM2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure CAN1 TX/RX pins: PB9 (TX), PB8 (RX) - AF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* Input Capture Callback */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (is_first_captured == 0)
        {
            ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            is_first_captured = 1;

            /* Switch polarity to capture falling edge (end of echo pulse) */
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            /* Restore polarity to rising for next measurement */
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

            capture_done = 1;
        }
    }
}

/* CAN RX callback - optional: prints received float (first 4 bytes) */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
    {
        return;
    }

    if (rxHeader.DLC >= 4)
    {
        float rcvd;
        memcpy(&rcvd, rxData, 4);
        uart_printf("CAN RX: %.2f cm (ID: 0x%03lX)\r\n", rcvd, (unsigned long)rxHeader.StdId);
    }
}

/* TIM2 IRQHandler - forward to HAL */
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}


/* DWT initialization for microsecond delay */
static void DWT_Delay_Init(void)
{
    /* Enable TRC */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* Reset the cycle counter */
    DWT->CYCCNT = 0;
    /* Enable the cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Delay in microseconds using cycle counter */
static void delay_us(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000L) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) { /* busy wait */ }
}

/* UART helper (printf style) */
static void uart_printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    /* Optionally blink an LED here to indicate error */
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can print or log file/line here */
}
#endif /* USE_FULL_ASSERT */
