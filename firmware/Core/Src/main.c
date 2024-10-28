/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Start_PWM(void);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  HAL_GPIO_WritePin(VLED_EN_GPIO_Port, VLED_EN_Pin, SET); //toggle VLED enable PB15
  Start_PWM();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  HAL_GPIO_WritePin(VLED_EN_GPIO_Port, VLED_EN_Pin, SET); //set VLED
//	  HAL_GPIO_WritePin(BRAKE_EN_GPIO_Port, BRAKE_EN_Pin, SET); //toggle Brake PB0
//	  HAL_GPIO_WritePin(BACKUP_EN_GPIO_Port, BACKUP_EN_Pin, SET); //toggle Brake PB0
//	  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, SET); //on board red LED PC13
//	  HAL_Delay(1100);
//
//	  HAL_GPIO_WritePin(BRAKE_EN_GPIO_Port, BRAKE_EN_Pin, RESET); //toggle Brake PB0
//	  HAL_GPIO_WritePin(BACKUP_EN_GPIO_Port, BACKUP_EN_Pin, RESET); //on board red LED PC13
//	  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, RESET);
//	  HAL_Delay(100);
	  //HAL_GPIO_TogglePin(VLED_EN_GPIO_Port, VLED_EN_Pin); //toggle VLED enable PB15
	  //HAL_Delay(100);
	  //HAL_GPIO_TogglePin(BRAKE_EN_GPIO_Port, BRAKE_EN_Pin);
	  HAL_Delay(500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 50);  //
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 50);
	  HAL_Delay(500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);  //
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);


	  //HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin); //on board red LED PC13
//	  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, RESET);
//	  HAL_GPIO_WritePin(VLED_EN_GPIO_Port, VLED_EN_Pin, RESET); //toggle Brake PB0
//	  HAL_GPIO_WritePin(BRAKE_EN_GPIO_Port, BRAKE_EN_Pin, RESET); //toggle Brake PB1
//	  HAL_Delay(100);
//	  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, SET); //on board red LED PC13
//	  HAL_GPIO_WritePin(BRAKE_EN_GPIO_Port, BRAKE_EN_Pin, SET); //toggle Brake PB0
//	  HAL_GPIO_WritePin(BACKUP_EN_GPIO_Port, BACKUP_EN_Pin, SET); //toggle Brake PB1
//	  HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RED_EN_Pin|GRN_EN_Pin|BLU_EN_Pin|VLED_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RED_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_SW_CS_Pin PA10 */
  GPIO_InitStruct.Pin = EXT_SW_CS_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_EN_Pin GRN_EN_Pin BLU_EN_Pin VLED_EN_Pin */
  GPIO_InitStruct.Pin = RED_EN_Pin|GRN_EN_Pin|BLU_EN_Pin|VLED_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_TURN_IN_Pin PB7 */
  GPIO_InitStruct.Pin = EXT_TURN_IN_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void checkLEDState(void){
	if(HAL_GPIO_ReadPin(BRAKE_EN_GPIO_Port, BRAKE_EN_Pin)){ //if brake light enabled:

	}

}

void Start_PWM(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // Start PWM for EXT_SW_EN
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  // Start PWM for BRAKE_EN
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  // Start PWM for BACKUP_EN
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

#ifdef  USE_FULL_ASSERT
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
