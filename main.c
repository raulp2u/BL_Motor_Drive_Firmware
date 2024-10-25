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
#define ACIONAMENTO_MALHA_ABERTA 	1
#define ACIONAMENTO_HALL 			0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

//Tabela de valores para seno

//#define TAMANHO_VETOR 6
//uint32_t VetorSeno[] = {0,0,1,2,4,6,9,13,17,22,27,33,39,46,53,61,69,78,88,98,108,119,131,143,155,168,182,196,210,225,241,257,273,290,307,325,343,362,381,401,421,441,462,483,505,527,549,572,595,619,642,667,691,716,741,767,793,819,846,872,899,927,954,982,1010,1039,1067,1096,1125,1154,1184,1213,1243,1273,1303,1334,1364,1395,1425,1456,1487,1518,1549,1580,1611,1643,1674,1705,1737,1768,1800,1831,1862,1894,1925,1956,1988,2019,2050,2081,2112,2143,2174,2204,2235,2265,2296,2326,2356,2386,2415,2445,2474,2503,2532,2560,2589,2617,2645,2672,2700,2727,2753,2780,2806,2832,2858,2883,2908,2932,2957,2980,3004,3027,3050,3072,3094,3116,3137,3158,3178,3198,3218,3237,3256,3274,3292,3309,3326,3342,3358,3374,3389,3403,3417,3431,3444,3456,3468,3480,3491,3501,3511,3521,3530,3538,3546,3553,3560,3566,3572,3577,3582,3586,3590,3593,3595,3597,3598,3599,3600,3599,3598,3597,3595,3593,3590,3586,3582,3577,3572,3566,3560,3553,3546,3538,3530,3521,3511,3501,3491,3480,3468,3456,3444,3431,3417,3403,3389,3374,3358,3342,3326,3309,3292,3274,3256,3237,3218,3198,3178,3158,3137,3116,3094,3072,3050,3027,3004,2980,2957,2932,2908,2883,2858,2832,2806,2780,2753,2727,2700,2672,2645,2617,2589,2560,2532,2503,2474,2445,2415,2386,2356,2326,2296,2265,2235,2204,2174,2143,2112,2081,2050,2019,1988,1956,1925,1894,1862,1831,1800,1768,1737,1705,1674,1643,1611,1580,1549,1518,1487,1456,1425,1395,1364,1334,1303,1273,1243,1213,1184,1154,1125,1096,1067,1039,1010,982,954,927,900,872,846,819,793,767,741,716,691,667,642,619,595,572,549,527,505,483,462,441,421,401,381,362,343,325,307,290,273,257,241,225,210,196,182,168,155,143,131,119,108,98,88,78,69,61,53,46,39,33,27,22,17,13,9,6,4,2,1,0};

uint32_t SetorLido=0;
int ValorMaximoTimer= 3600;
unsigned int DutyCycle=3000;
char* mensagem = "Olá Mundo\n";
volatile uint32_t t_us = 0;  // Variável que recebe o tempo decorrido em microssegundos
volatile int ok = 1;  // Controle do fluxo
volatile int hallint = 0;  // Valor do hall sensor
volatile int SetorAcionado = 0;  // Setor acionado com base no valor de hallint
volatile uint32_t tempo_entre_setores = 0;  // Tempo entre cada troca de setor
volatile int setores_acionados = 0b000000;  // Controle dos setores acionados (flags para cada setor)
volatile uint32_t soma_tempo_trocas = 0;  // Soma do tempo entre as trocas de setores
volatile int contador_trocas = 0;  // Contador de trocas de setores

int hall[3];
int ValorContador=1800;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void MX_USART2_UART_Init(void);
void delay_ms(uint32_t ms);
void SysTick_Handler(void);
void reset_systick(void);
void checar_setores(int setor);
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
  /* USER CODE BEGIN 2 */
  // Configuração do SysTick para interromper a cada 1 microsegundo
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 72);

#ifdef ACIONAMENTO_MALHA_ABERTA
   	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	//HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	//HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	//HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	TIM1->CCR1=ValorMaximoTimer;//Desliga PWM
	TIM1->CCR2=ValorMaximoTimer;//Desliga PWM
	TIM1->CCR3=ValorMaximoTimer;//Desliga PWM

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);//Desliga Low CH1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);//Desliga Low CH2
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);//Desliga Low CH3
#endif

   HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Reseta o SysTick e t_us antes de iniciar as mensagens
	         reset_systick();

	         // Enviar mensagem "Olá Mundo" três vezes, uma vez por segundo usando a contagem por t_us
	         for (int i = 0; i < 3; i++) {
	             HAL_UART_Transmit(&huart2, (uint8_t*)mensagem, strlen(mensagem), HAL_MAX_DELAY);
	             uint32_t start_time = t_us;
	             while ((t_us - start_time) < 1000000);  // Atraso de 1 segundo
	         }

	  //Faz a leitura dos sensores Hall
	  hall[0]= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	  hall[1]= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)*10;
	  hall[2]= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)*100;
	  hallint=  hall[0]+hall[1]+hall[2];//Monta o código dos sensores Hall

	 //Escolhe o setor que vai acionar
	 //de acordo com a leitura do Hall
	  // Verifica a variável 'ok'
	          if (ok == 0) {
	              char* aguardando_msg = "Programa aguardando atualização de variáveis...\n";
	              HAL_UART_Transmit(&huart2, (uint8_t*)aguardando_msg, strlen(aguardando_msg), HAL_MAX_DELAY);

	              // Reseta o SysTick e t_us ao entrar no estado de espera
	              reset_systick();
	              setores_acionados = 0b000000;  // Reinicia controle de setores
	              soma_tempo_trocas = 0;  // Reseta a soma do tempo
	              contador_trocas = 0;  // Reseta o contador de trocas

	             // verifica continuamente `hallint` e atualiza `SetorAcionado`
	              while (ok == 1) {
	                  uint32_t inicio_troca = t_us;
	                  switch (hallint) {
	                      case 1:
	                          SetorAcionado = 4;
	                          checar_setores(4);
	                          break;
	                      case 10:
	                          SetorAcionado = 6;
	                          checar_setores(6);
	                          break;
	                      case 11:
	                          SetorAcionado = 5;
	                          checar_setores(5);
	                          break;
	                      case 100:
	                          SetorAcionado = 2;
	                          checar_setores(2);
	                          break;
	                      case 101:
	                          SetorAcionado = 3;
	                          checar_setores(3);
	                          break;
	                      case 110:
	                          SetorAcionado = 1;
	                          checar_setores(1);
	                          break;
	                      default:
	                          SetorAcionado = 0;
	                          break;
	                  }
	                  tempo_entre_setores = t_us - inicio_troca;
	                  soma_tempo_trocas += tempo_entre_setores;  // Adiciona ao tempo total
	                  contador_trocas++;  // Incrementa o contador de trocas

	                  // Checa se todos os setores foram acionados (todos os bits de `setores_acionados` devem estar em 1)
	                  if (setores_acionados == 0b111111) {


	                      // Calcula o tempo médio
	                      float tempo_medio = (contador_trocas > 0) ? (soma_tempo_trocas / (float)contador_trocas) / 1000000.0f : 0.0f; // em segundos
	                      char buffer[100];
	                      snprintf(buffer, sizeof(buffer), "O tempo médio entre a troca de setores foi de %.6f segundos\n", tempo_medio);
	                      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

	                      break;
	                  }

	                  //Codigo de acionamento do motor por setores

	                  if(SetorAcionado == 0) //Setor 0 -> Desliga tudo
	                  		{
	                  			TIM1->CCR1=ValorMaximoTimer;//Desliga PWM
	                  			TIM1->CCR2=ValorMaximoTimer;//Desliga PWM
	                  			TIM1->CCR3=ValorMaximoTimer;//Desliga PWM
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);//Desliga Low CH1
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);//Desliga Low CH2
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);//Desliga Low CH3
	                  		}

	                  		if(SetorAcionado == 1) //Setor 1 -> High "C" e Low "A"
	                  		{
	                  			//Lê no Hall 100
	                  			TIM1->CCR1=ValorMaximoTimer;//Desliga PWM
	                  			TIM1->CCR2=ValorMaximoTimer;//Desliga PWM
	                  			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);//Desliga Low CH1
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);//Desliga Low CH2
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);//Desliga Low CH3

	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);//Liga Low CH1
	                  			TIM1->CCR3=DutyCycle;//VetorSeno[flagA];
	                  		}

	                  		if(SetorAcionado == 2) //Setor 2 - High "C" - Low "B"
	                  		{
	                  			//Lê no Hall 101
	                  			TIM1->CCR1=ValorMaximoTimer;//Desliga PWM
	                  			TIM1->CCR2=ValorMaximoTimer;//Desliga PWM
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);//Desliga Low CH1
	                  			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);//Desliga Low CH2
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);//Desliga Low CH3

	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_RESET);//Liga Low CH2
	                  			TIM1->CCR3=DutyCycle;//VetorSeno[flagA];
	                  		}

	                  		if(SetorAcionado == 3) //Setor 3 - High "A" - Low "B"
	                  		{
	                  			//Lê no Hall 001
	                  			TIM1->CCR2=ValorMaximoTimer;//Desliga PWM
	                  			TIM1->CCR3=ValorMaximoTimer;//Desliga PWM
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);//Desliga Low CH1
	                  			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);//Desliga Low CH2
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);//Desliga Low CH3

	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_RESET);//Liga Low CH2
	                  			TIM1->CCR1=DutyCycle;//VetorSeno[flagA];
	                  		}

	                  		if(SetorAcionado == 4) //Setor 4 - High "A" - Low "C"
	                  		{
	                  			//Lê no Hall 011
	                  			TIM1->CCR2=ValorMaximoTimer;//Desliga PWM
	                  			TIM1->CCR3=ValorMaximoTimer;//Desliga PWM
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);//Desliga Low CH1
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);//Desliga Low CH2
	                  			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);//Desliga Low CH3

	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET);//Liga Low CH3
	                  			TIM1->CCR1=DutyCycle;//VetorSeno[flagA];
	                  		}

	                  		if(SetorAcionado == 5) //Setor 5 - High "B" - Low "C"
	                  		{
	                  			//Lê no Hall 010
	                  			TIM1->CCR1=ValorMaximoTimer;//Desliga PWM
	                  			TIM1->CCR3=ValorMaximoTimer;//Desliga PWM
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);//Desliga Low CH1
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);//Desliga Low CH2
	                  			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);//Desliga Low CH3

	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET);//Liga Low CH3
	                  			TIM1->CCR2=DutyCycle;//VetorSeno[flagA];
	                  		}

	                  		if(SetorAcionado == 6) //Setor 6 - High "B" - Low "A"
	                  		{
	                  			//Lê no Hall 110
	                  			TIM1->CCR1=ValorMaximoTimer;//Desliga PWM
	                  			TIM1->CCR3=ValorMaximoTimer;//Desliga PWM
	                  			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);//Desliga Low CH1
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);//Desliga Low CH2
	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);//Desliga Low CH3

	                  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);//Liga Low CH1
	                  			TIM1->CCR2=DutyCycle;//VetorSeno[flagA];
	                  		}
	              }

	              // Reseta o SysTick e t_us quando ok é alterado para 1
	              reset_systick();
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
  sConfigOC.Pulse = 3599;
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
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LowCH1_Pin|LowCH2_Pin|LowCH3_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : LowCH1_Pin LowCH2_Pin LowCH3_Pin */
  GPIO_InitStruct.Pin = LowCH1_Pin|LowCH2_Pin|LowCH3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Função para resetar o SysTick e t_us
void reset_systick(void) {
    t_us = 0;
    SysTick->VAL = 0;  // Reseta o contador do SysTick para zero
}

// Função para marcar setor acionado e verificar setores
void checar_setores(int setor) {
    switch (setor) {
        case 1:
            setores_acionados |= 0b000001;
            break;
        case 2:
            setores_acionados |= 0b000010;
            break;
        case 3:
            setores_acionados |= 0b000100;
            break;
        case 4:
            setores_acionados |= 0b001000;
            break;
        case 5:
            setores_acionados |= 0b010000;
            break;
        case 6:
            setores_acionados |= 0b100000;
            break;
        default:
            break;
    }
}

// Manipulador da interrupção SysTick
void SysTick_Handler(void) {
    HAL_IncTick();
    t_us++;  // Incrementa o tempo em microssegundos
}
static void MX_USART2_UART_Init(void) {
    // Configuração do UART2 a 115200 baud rate
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        // Inicialização do UART falhou
        Error_Handler();
    }
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
