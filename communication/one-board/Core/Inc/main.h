/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* CubeMX defines these in main.c but declares them nowhere, so every module
 * that touches a peripheral would otherwise need its own extern. Declaring
 * them here once keeps node1.c, node2sim.c, lcd.c and uart_log.c free of
 * duplicated declarations that could drift out of step with main.c. */
extern CAN_HandleTypeDef  hcan1;
extern CAN_HandleTypeDef  hcan2;
extern SPI_HandleTypeDef  hspi1;
extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart3;

/* Filter configuration plus HAL_CAN_Start() plus RX notification, for both
 * controllers. CubeMX generates MX_CAN1_Init()/MX_CAN2_Init(), which only
 * configure bit timing - a bxCAN cell with no filter accepts nothing at
 * all, and an unstarted one transmits nothing, so this has to exist
 * separately. Also called by the recovery paths in node1.c/node2sim.c
 * after they re-init a controller that failed to leave initialisation
 * mode. */
void Can_Setup(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
