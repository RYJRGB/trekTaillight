/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RED_LED_Pin GPIO_PIN_13
#define RED_LED_GPIO_Port GPIOC
#define EXT_SW_EN_Pin GPIO_PIN_6
#define EXT_SW_EN_GPIO_Port GPIOA
#define EXT_SW_CS_Pin GPIO_PIN_7
#define EXT_SW_CS_GPIO_Port GPIOA
#define BRAKE_EN_Pin GPIO_PIN_0
#define BRAKE_EN_GPIO_Port GPIOB
#define BACKUP_EN_Pin GPIO_PIN_1
#define BACKUP_EN_GPIO_Port GPIOB
#define RED_EN_Pin GPIO_PIN_2
#define RED_EN_GPIO_Port GPIOB
#define GRN_EN_Pin GPIO_PIN_10
#define GRN_EN_GPIO_Port GPIOB
#define BLU_EN_Pin GPIO_PIN_11
#define BLU_EN_GPIO_Port GPIOB
#define VLED_EN_Pin GPIO_PIN_15
#define VLED_EN_GPIO_Port GPIOB
#define EXT_TURN_IN_Pin GPIO_PIN_6
#define EXT_TURN_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
