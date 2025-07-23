#include "stm32f7xx.h"
#include "main.h"
/*currently number of times pressed within 5s blinks led for that number*/
/*number of times push button pressed, led blinks with that delay in seconds*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
volatile uint32_t count=0;
volatile uint32_t led_blink_trigger = 0 ;
volatile uint32_t pressed =0;

void led_blink(uint32_t blink_time);
int main()
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	GPIOD->ODR |= (1 <<5);
	while(1)
	{
		if(count>0)
		{
			led_blink(count);
			count=0;
		}
		HAL_Delay(5000);
	}
}
void EXTI15_10_IRQHandler(void)
{
	if(EXTI->PR & EXTI_PR_PR11){ //CHECKS WHETHER INTERRUPT TRIGGERED, PR REGISTER HAS PENDING INTERRUPT FLAGS FOR ALL EXTI LINES
	EXTI->PR = EXTI_PR_PR11;//CLEAR INTERRUPT FLAG
	if ((GPIOI->IDR & (1 << 11))==0)
	        {
				pressed=1;
		   }else
		   {
			   if(pressed ==1)
			   {
				   pressed =0;
				   count ++;

			   }
		   }
	}
}
void led_blink(uint32_t blink_time)
{
	for(int i=0; i<=blink_time; i++)
	{
		GPIOD->ODR &= ~(1 <<5);  //led on
		HAL_Delay(count * 1000);
		GPIOD->ODR |= (1 << 5); // led off
		HAL_Delay(count * 1000);
	}
}




void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PI11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING ; // <------------Mode change for interrupt configuration
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);


  /* USER CODE BEGIN MX_GPIO_Init_2 */
  SYSCFG->EXTICR[2] &= ~(0xF << 12);   // Clear EXTI11 bits, OXF IS GOOD PRACTISE TO CLEAR BITS
      SYSCFG->EXTICR[2] |=  (0x8 << 12);   // Set EXTI11 to Port I, 0X8 IS GOOD PRACTISE TO SET BITS AS ALL BITS NEED TO BE OPERATED TO PREVENT MALFUNCITON

      EXTI->IMR  |= (1 << 11);             // Interrupt request from line 11 is not masked, ALLOWING IT TO GENERATE INTERRUPT, IF NOT DONE NOTHING TRIGGERS
      EXTI->RTSR |= (1 << 11);             // Rising trigger enabled (for Event and Interrupt) for input line
      EXTI->FTSR |= (1 << 11);			// Falling edge trigger enabled

      // Enable NVIC for EXTI15_10
      NVIC_EnableIRQ(EXTI15_10_IRQn);  //Activates the interrupt handler for lines 10–15
      NVIC_SetPriority(EXTI15_10_IRQn, 1);
  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  { GPIOD->ODR &= ~(1 << 5);
  	  HAL_Delay(500);
  	  GPIOD->ODR |= (1 << 5);
  	  HAL_Delay(500);
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


