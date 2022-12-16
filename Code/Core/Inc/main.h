/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CTL2_Pin GPIO_PIN_1
#define CTL2_GPIO_Port GPIOB
#define CTL1_Pin GPIO_PIN_2
#define CTL1_GPIO_Port GPIOB
#define REL_Pin GPIO_PIN_10
#define REL_GPIO_Port GPIOB
#define SELA_Pin GPIO_PIN_11
#define SELA_GPIO_Port GPIOB
#define SELB_Pin GPIO_PIN_12
#define SELB_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_11
#define LED0_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOA
#define BMS_ALERT_Pin GPIO_PIN_15
#define BMS_ALERT_GPIO_Port GPIOA
#define MCU_HOLD_Pin GPIO_PIN_3
#define MCU_HOLD_GPIO_Port GPIOB
#define BMS_ON_Pin GPIO_PIN_4
#define BMS_ON_GPIO_Port GPIOB
#define MCU_EN_Pin GPIO_PIN_5
#define MCU_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
