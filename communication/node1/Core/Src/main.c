/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "node1.h"
#include "uart_log.h"
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

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CAN_FilterTypeDef   CAN1_sFilterConfig;

/* Packed HAL return codes from MX_CAN1_Setup(), reported at startup:
 * bits 1:0 = ConfigFilter, bits 3:2 = Start, bits 5:4 = ActivateNotification.
 * 0 means HAL_OK. The generated skeleton discards these entirely, which is
 * how a failed HAL_CAN_Start() can go unnoticed. */
uint8_t g_Can1SetupStatus;

TIM_HandleTypeDef htim2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void MX_CAN1_Setup(void);
static void MX_TIM2_Init(void);
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_CAN1_Setup();
  UartLog_Init();   /* before Node1_Init(): it prints its startup banner immediately */
  Node1_Init();     /* brings up the LCD too - its one-off screen clear blocks,
                     * so it must finish before the periodic timer starts */

  MX_TIM2_Init();
  HAL_TIM_Base_Start_IT(&htim2);   /* fires Node1_OnTimerTick() every 50 ms */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Node1_Periodic();
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
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
#if NODE1_SELFTEST == 1
  /* Self-test 1: SILENT loopback, not plain loopback. Plain CAN_MODE_LOOPBACK
   * still leaves CANRX wired to the pin, so with no powered transceiver the
   * floating input can read dominant and the cell never sees the 11 recessive
   * bits it needs to leave initialisation mode - INAK stays set, HAL_CAN_Start()
   * times out and every AddTxMessage then fails. Silent loopback disconnects
   * CANRX internally and holds CANTX recessive: the "hot self-test" mode, which
   * needs no transceiver, no bus and no wiring at all. */
  hcan1.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
#else
  /* Normal: a real second node is on the bus, so CAN1 transmits for real. */
  hcan1.Init.Mode = CAN_MODE_NORMAL;
#endif
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  /* Bit timing: 500 kbit/s. PCLK1=42MHz, NBT = 1(Sync)+10(BS1)+3(BS2) = 14 TQ.
   * BaudRate = 42e6 / (6 * 14) = 500000 bit/s. SamplePoint = 11/14 = 78.6%
   * (dai cho phep 75-82%). SJW=2TQ (2-3). */
  hcan1.Init.Prescaler     = 6;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1      = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2      = CAN_BS2_3TQ;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC4 PC5 PC6
                           PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
#if (NODE1_USE_LCD == 0)
  /* Park PB6 high.
   *
   * CAN2 is never initialised in this practice, so without this the pin
   * floats and the on-board CAN2 transceiver can drive the shared bus
   * permanently dominant - which lands right back on CAN1_RX and stops
   * CAN1 ever leaving initialisation mode. The bus has to be idle *before*
   * CAN1 starts, so the pin is parked here, in GPIO init.
   *
   * PB6 is CAN2_TX (and, on this board, also LCD_BL). This practice never
   * initialises CAN2, so without this the pin is left floating - and a CAN
   * transceiver whose TXD input floats can sit on the bus driving it
   * permanently dominant. If CAN1 and CAN2 share a bus, that dominant level
   * comes straight back into CAN1_RX, and CAN1 then never sees the 11
   * recessive bits it needs to leave initialisation mode: every transmission
   * fails while every other part of the firmware looks perfectly healthy.
   *
   * High = recessive = transceiver idle, which is also backlight-on for the
   * LCD, so the two uses of this pin want the same level. Skipped when the
   * LCD driver owns the pin instead. */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  GPIO_InitStruct.Pin   = GPIO_PIN_6;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void MX_CAN1_Setup(void)
{
	/* Chi 1 CAN peripheral (CAN1) tren board nay, khong co CAN2 lam slave
	 * nen dung het 28 filter bank cho CAN1. Mask=0 -> nhan moi ID, loc that
	 * su (chi quan tam CAN012_ID_RX) nam trong phan mem, xem Node1_OnCanRx(). */
	CAN1_sFilterConfig.FilterBank           = 0;
	CAN1_sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
	CAN1_sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
	CAN1_sFilterConfig.FilterIdHigh         = 0x0000;
	CAN1_sFilterConfig.FilterIdLow          = 0x0000;
	CAN1_sFilterConfig.FilterMaskIdHigh     = 0x0000;
	CAN1_sFilterConfig.FilterMaskIdLow      = 0x0000;
	CAN1_sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN1_sFilterConfig.FilterActivation     = ENABLE;
	CAN1_sFilterConfig.SlaveStartFilterBank = 14;

	g_Can1SetupStatus  = (uint8_t)HAL_CAN_ConfigFilter(&hcan1, &CAN1_sFilterConfig);
	g_Can1SetupStatus |= (uint8_t)(HAL_CAN_Start(&hcan1) << 2);
	g_Can1SetupStatus |= (uint8_t)(HAL_CAN_ActivateNotification(&hcan1,
	                                   CAN_IT_RX_FIFO0_MSG_PENDING) << 4);
}

/**
  * @brief TIM2 Initialization Function - 50 ms periodic tick for the 0x012 send.
  * Not part of the generated .ioc (hand-added, so it lives entirely in
  * USER CODE to survive CubeMX regeneration).
  * TIM2CLK = 84 MHz (APB1 timer clock, since APB1CLKDivider != 1).
  * 84e6 / (8399+1) = 10 kHz counter clock; / (499+1) = 20 Hz -> period 50.000 ms.
  * @retval None
  */
static void MX_TIM2_Init(void)
{
	__HAL_RCC_TIM2_CLK_ENABLE();

	htim2.Instance               = TIM2;
	htim2.Init.Prescaler         = 8399;
	htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim2.Init.Period            = 499;
	htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
