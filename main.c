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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

//Tabela de valores para seno

//#define TAMANHO_VETOR 6
//uint32_t VetorSeno[] = {1800,	0,		0,		1800,	3600,	3600};
//uint32_t flagA=0,flagB=2,flagC=4;

//uint32_t VetorSeno[] = {1800,1831,1862,1894,1925,1956,1988,2019,2050,2081,2112,2143,2174,2204,2235,2265,2296,2326,2356,2386,2415,2445,2474,2503,2532,2560,2589,2617,2645,2672,2700,2727,2753,2780,2806,2832,2858,2883,2908,2932,2957,2980,3004,3027,3050,3072,3094,3116,3137,3158,3178,3198,3218,3237,3256,3274,3292,3309,3326,3342,3358,3374,3389,3403,3417,3431,3444,3456,3468,3480,3491,3501,3511,3521,3530,3538,3546,3553,3560,3566,3572,3577,3582,3586,3590,3593,3595,3597,3598,3599,3600,3599,3598,3597,3595,3593,3590,3586,3582,3577,3572,3566,3560,3553,3546,3538,3530,3521,3511,3501,3491,3480,3468,3456,3444,3431,3417,3403,3389,3374,3358,3342,3326,3309,3292,3274,3256,3237,3218,3198,3178,3158,3137,3116,3094,3072,3050,3027,3004,2980,2957,2932,2908,2883,2858,2832,2806,2780,2753,2727,2700,2672,2645,2617,2589,2560,2532,2503,2474,2445,2415,2386,2356,2326,2296,2265,2235,2204,2174,2143,2112,2081,2050,2019,1988,1956,1925,1894,1862,1831,1800,1768,1737,1705,1674,1643,1611,1580,1549,1518,1487,1456,1425,1395,1364,1334,1303,1273,1243,1213,1184,1154,1125,1096,1067,1039,1010,982,954,927,900,872,846,819,793,767,741,716,691,667,642,619,595,572,549,527,505,483,462,441,421,401,381,362,343,325,307,290,273,257,241,225,210,196,182,168,155,143,131,119,108,98,88,78,69,61,53,46,39,33,27,22,17,13,9,6,4,2,1,0,0,0,1,2,4,6,9,13,17,22,27,33,39,46,53,61,69,78,88,98,108,119,131,143,155,168,182,196,210,225,241,257,273,290,307,325,343,362,381,401,421,441,462,483,505,527,549,572,595,619,642,667,691,716,741,767,793,819,846,872,899,927,954,982,1010,1039,1067,1096,1125,1154,1184,1213,1243,1273,1303,1334,1364,1395,1425,1456,1487,1518,1549,1580,1611,1643,1674,1705,1737,1768};
//uint32_t flagA=0,	flagB=120,	flagC=240;

//uint32_t FaseB[] = {3600,	1800,	0,		0,		1800,	3600};
//uint32_t FaseC[] = {3600,	3600,	1800,	0,		0,		1800};


//Velocidade
uint32_t velocidade=50;
uint32_t setor;
int hall[3];
int hallint;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

   HAL_TIM_Base_Start(&htim1);
   //HAL_TIM_Base_Start(&htim2);

   //Inicia Timer 2: Que dita a senoide
//   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
 //  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  hall[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	  hall[1]= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)*10;
	  hall[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)*100;
	  hallint=  hall[0]+hall[1]+hall[2];

	 switch (hallint)
	 	{
	 case 1:
		{
		 setor=2;
		   HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		   HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		   TIM1->CCR2=100;
		   TIM1->CCR3=3500;

		   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

		   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);



		 break;
		}
	 case 10:
		{
		 setor=4;
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
		  TIM1->CCR1=100;
		  TIM1->CCR2=3500;
		   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

		   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);


		 break;
		}
	 case 11:
		{
		 setor=3;
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

		  TIM1->CCR1=100;
		  TIM1->CCR3=3500;

		   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

		   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

		   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

		 break;
		}
	 case 100:
		{
		 setor=6;
		   HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		   HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		   TIM1->CCR3=100;
		   TIM1->CCR1=3500;

		   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

		   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

		   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);


		 break;
		}
	 case 101:
		{
		 setor=1;
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
   	   	  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

   	   	  TIM1->CCR2=100;
   	   	  TIM1->CCR1=3500;

		   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

		   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);



		 break;
		}
	 case 110:
		{
		 setor=5;
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		  TIM1->CCR3=100;
		  TIM1->CCR2=3500;


		   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

		   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

		 break;
		}

	 default:{
		 	 	 setor=0;
		 	 	 break;
	 	 	 }
	 }


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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 3599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 10;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 719;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
