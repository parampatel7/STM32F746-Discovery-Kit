/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include"stm32f7xx.h"
//HAL: interrupt of timer every 5s
//Baremetal same code


///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//
//
//TIM_HandleTypeDef htim6;
//
///* USER CODE BEGIN PV */
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_TIM6_Init(void);
///* USER CODE BEGIN PFP */
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_TIM6_Init();
//  /* USER CODE BEGIN 2 */
//
//  /* USER CODE END 2 */
//  //Start timer in interrupt mode
//  HAL_TIM_Base_Start_IT(&htim6);
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 25;
//  RCC_OscInitStruct.PLL.PLLN = 400;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 9;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Activate the Over-Drive mode
//  */
//  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief TIM6 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM6_Init(void)
//{
//
//  /* USER CODE BEGIN TIM6_Init 0 */
//
//  /* USER CODE END TIM6_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM6_Init 1 */
//
//  /* USER CODE END TIM6_Init 1 */
//  htim6.Instance = TIM6;
//  htim6.Init.Prescaler = 10799;
//  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim6.Init.Period = 49999;
//  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM6_Init 2 */
//
//  /* USER CODE END TIM6_Init 2 */
//
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  /* USER CODE BEGIN MX_GPIO_Init_1 */
//
//  /* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : PD5 */
//  GPIO_InitStruct.Pin = GPIO_PIN_5;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
////----------------nvic ---------------------
//  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
//  /* USER CODE BEGIN MX_GPIO_Init_2 */
//
//  /* USER CODE END MX_GPIO_Init_2 */
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  *
//  *
//  */
////Interrupt callback function by HAL. Automatically called when TIM6 overflows.
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  if (htim->Instance == TIM6)
//  {
//    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_5);
//  }
//}
//
///**
//  * @brief TIM6 IRQ Handler
//  */
//void TIM6_DAC_IRQHandler(void)
//{
//  HAL_TIM_IRQHandler(&htim6);
//}
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//#ifdef USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
//

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void TIM6_Init_BareMetal(void);

// Entry point
int main(void)
{
  HAL_Init();                  // Optional if other HAL components needed
  SystemClock_Config();        // Clock setup
  MX_GPIO_Init();              // GPIO setup for PD5
  TIM6_Init_BareMetal();       // Timer setup (bare-metal)

  while (1)
  {
    // All blinking handled in interrupt
  }
}

// System Clock Configuration — unchanged
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
}

// Bare-metal Timer 6 Initialization
void TIM6_Init_BareMetal(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // Enable TIM6 clock

  TIM6->PSC = 53999;     // Prescaler
  TIM6->ARR = 9999;      // Auto-reload (Overflow every ~5s)

  TIM6->DIER |= TIM_DIER_UIE;   // Enable Update Interrupt
  TIM6->CR1 |= TIM_CR1_CEN;     // Start Timer

  NVIC_SetPriority(TIM6_DAC_IRQn, 0);    // IRQ priority
  NVIC_EnableIRQ(TIM6_DAC_IRQn);        // Enable TIM6 interrupt
}

// GPIO initialization — unchanged
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

// TIM6 interrupt handler (bare-metal)
void TIM6_DAC_IRQHandler(void)
{
  if (TIM6->SR & TIM_SR_UIF)  // Check Update Interrupt Flag
  {
    TIM6->SR &= ~TIM_SR_UIF;  // Clear flag
    GPIOD->ODR ^= GPIO_ODR_OD5; // Toggle PD5
  }
}
