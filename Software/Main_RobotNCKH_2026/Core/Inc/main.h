/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_9
#define IN1_GPIO_Port GPIOI
#define IN2_Pin GPIO_PIN_10
#define IN2_GPIO_Port GPIOI
#define IN3_Pin GPIO_PIN_11
#define IN3_GPIO_Port GPIOI
#define T5C3_MCLK_CAM_Pin GPIO_PIN_12
#define T5C3_MCLK_CAM_GPIO_Port GPIOH
#define LCD_RESX_Pin GPIO_PIN_15
#define LCD_RESX_GPIO_Port GPIOH
#define LCD_DCX_Pin GPIO_PIN_0
#define LCD_DCX_GPIO_Port GPIOI
#define LCD_CSX_Pin GPIO_PIN_1
#define LCD_CSX_GPIO_Port GPIOI
#define CAM_PWDN_Pin GPIO_PIN_2
#define CAM_PWDN_GPIO_Port GPIOD
#define CAM_RET_Pin GPIO_PIN_3
#define CAM_RET_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
