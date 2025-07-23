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
#include "stm32f7xx.h"
/*USER PUSH BUTTON PRESS WILL TURN ON LED*/
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void GPIO_Init(void);
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
  // Adjust if using a different CMSIS core header

    GPIO_Init();

    while (1) {
        /*if ((GPIOC->IDR & (1 << 13)) == 0) { // If button pressed (active low)
            GPIOA->ODR |= (1 << 5);          // Turn on LED (set PA5)
        } else {
            GPIOA->ODR &= ~(1 << 5);         // Turn off LED (reset PA5)
        }*/
//IDR is input data register, for PI11 we read 11th bit
    	uint32_t button_state = (GPIOI->IDR & (1U << 11)); //1U AS UNSIGNED, PLACING ONLY 1 IS INT AND CAN CAUSE SOME UNDEFINED BEHAVIOUR

    	        if (button_state)
    	        {//ODR is output data register, PD5 has LED
    	            // Button is high (not pressed if active-low)
    	            GPIOD->ODR &= ~(1U << 5);  // Turn off LED
    	        }
    	        else
    	        {
    	            // Button is low (pressed if active-low)
    	            GPIOD->ODR |= (1U << 5);   // Turn on LED
    	        }
    }
}
static void GPIO_Init(void) {
    // 1. Enable clock for GPIOA and GPIOC
    /*RCC->AHB1ENR |= (1 << 0);   // GPIOA clock enable
    RCC->AHB1ENR |= (1 << 2);   // GPIOC clock enable

    // 2. Configure PA5 as general purpose output
    GPIOA->MODER &= ~(3 << (5 * 2));  // Clear mode bits for PA5
    GPIOA->MODER |=  (1 << (5 * 2));  // Set mode to 01 (general purpose output)

    GPIOA->OTYPER &= ~(1 << 5);       // Set output type to push-pull
    GPIOA->OSPEEDR |= (3 << (5 * f2)); // High speed (optional)
    GPIOA->PUPDR &= ~(3 << (5 * 2));  // No pull-up, pull-down

    // 3. Configure PC13 as input
    GPIOC->MODER &= ~(3 << (13 * 2)); // Input mode (00)
    GPIOC->PUPDR &= ~(3 << (13 * 2)); // No pull-up, pull-down*/

	// Enable clocks for GPIOD and GPIOI
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOIEN;

	    // --- Configure PD5 (LED) ---
	     /*MODER5 = 00 (INPUT)
	    			01 (output)
	    			10 (ALTERNATE FUNCTION
	    			11 (ANALOG) */
	    GPIOD->MODER &= ~(3U << 10); //WE FIRST CLEAR ALL TWO BITS AND THEN SET THEM
	    GPIOD->MODER |=  (1U << 10);

	    // OTYPER5 = 0 (push-pull)
	    GPIOD->OTYPER &= ~(1U << 5);

	    // OSPEEDR5 = 10 (medium speed) or 11 (high speed)
	    GPIOD->OSPEEDR &= ~(3U << (5 * 2));
	    GPIOD->OSPEEDR |=  (2U << (5 * 2));  // Medium speed

	    // PUPDR5 = 00 (no pull-up/pull-down)
	    GPIOD->PUPDR &= ~(3U << 10);

	    // --- Configure PI11 (Button) ---
	    // MODER11 = 00 (input)
	    GPIOI->MODER &= ~(3U << 22);

	    // PUPDR11 = 01 (pull-up), assuming active-low button
	    /*  00	No Pull	Pin is floating (default)
			01	Pull-Up	Pin is pulled high, CONNECTED TO Vcc
			10	Pull-Down	Pin is pulled low, CONNECTED TO GROUND
			11	Reserved	Do not use*/
	    GPIOI->PUPDR &= ~(3U << 22);
	    GPIOI->PUPDR |=  (1U << 22);  // Enable pull-up

	    // OTYPER and OSPEEDR not needed for input pins
}

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */


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
