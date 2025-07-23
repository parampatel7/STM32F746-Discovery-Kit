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
/* USER CODE BEGIN PFP */
static void MX_GPIO_Init(void);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
MX_GPIO_Init();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
uint8_t buttonstate = 1;
uint8_t lastbuttonstate = 1;
uint8_t ledstate = 0;
  while (1)
  {
    /* USER CODE END WHILE */
	  buttonstate = (GPIOI->IDR & (1U << 11)) ? 1 : 0;//conditional operator, of true first condition
	 	  if(buttonstate == 0 && lastbuttonstate == 1)//detect rising edge of push button
	 	  {
	 		  ledstate =! ledstate;//toggle led state if rising edge detected

	 	  	  if(ledstate)//LED is logic low
	 	  	  {
	 	  		  GPIOD-> ODR &= ~(1U << 5);//clear bit of output data register, to turn ON LED
	 	  	  }
	 	  	  else
	 	  	  {
	 	  		  GPIOD-> ODR |= (1U << 5);//set bit of ORD to turn OFF LED
	 	  	  }
	 	  HAL_Delay(200); //DEBOUNCE
	 	  }
	 	  lastbuttonstate = buttonstate;
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


static void MX_GPIO_Init(void){ //AHB CONNECT CPU TO MEMORY AND PERIPHERALS
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOIEN;

	GPIOD->MODER &= ~(3U << 10); //10 BECAUSE WE CONFIGURE THE PD5 PIN, HENCE MODE5
	GPIOD->MODER |= (1U << 10);

	GPIOD->OSPEEDR&= ~(3U << 10);//OUTPUT SPEED REGISTER
	GPIOD->OSPEEDR |= (1U <<10);

	GPIOD-> PUPDR &= ~(3U << 10);

	GPIOI->MODER &= ~(3U << 22); //22 BECAUSE WE CONFIGURE THE PI11 PIN, HENCE MODE5
	GPIOI->PUPDR &= ~(3U <<22);
	GPIOI->PUPDR |= (1U <<22);

}
/**
  * @brief System Clock Configuration
  * @retval None
  */



void SystemClock_Config(void) {
    // 1. Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));  // Wait until HSE is ready

    // 2. Configure Power Regulator
    /*The STM32's power control system (voltage scaling, overdrive mode, etc.
     is controlled by the PWR peripheral, but it is disabled by default to save power.
     So before you can use it, you must enable its clock on the APB1 bus.*/

    // 3. Enable overdrive mode from power control registers (needed for 216 MHz)
    /*ODEN This bit enables the OverDrive mode, which is required when running the core at frequencies above 180 MHz
     The internal voltage regulator must supply more power to the CPU when running at high speeds.
OverDrive mode boosts the regulator’s output to ensure stable operation at high frequencies.*/
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR1 |= PWR_CR1_ODEN;  // Overdrive enable
    while (!(PWR->CSR1 & PWR_CSR1_ODRDY));//wait till overdrive is ready

    PWR->CR1 |= PWR_CR1_ODSWEN;  // Switch Overdrive,/It starts the transition from regular voltage regulator mode to OverDrive mode.
    //This is necessary to fully apply the OverDrive settings and allow the system clock to run at up to 216 MHz
    while (!(PWR->CSR1 & PWR_CSR1_ODSWRDY)); // Wait till OD switch ready

    // 4. Configure Flash latency(7 wait states for 216 MHz)
    /*The Flash access control register is used to enable/disable the acceleration features and
   control the Flash memory access time according to CPU frequency.*/
    FLASH->ACR = FLASH_ACR_LATENCY_7WS | FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;//latency for 216MHz, Enable Prefetch, Enable Adaptice Real Time accelerator

    // 5. Configure PLL
    RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos)  | // PLLM = 25
                   (432 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 432
                   (0 << RCC_PLLCFGR_PLLP_Pos)   | // PLLP = 2 (00 = /2)
                   (RCC_PLLCFGR_PLLSRC_HSE)      |
                   (9 << RCC_PLLCFGR_PLLQ_Pos);    // PLLQ = 9

    // 5. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));  // Wait for PLL to lock

    // 6. Configure clock dividers, Set AHB, APB1, and APB2 prescalers before switching system clock
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 |    // AHB = SYSCLK / 1
                RCC_CFGR_PPRE1_DIV4 |   // APB1 = AHB / 4
                RCC_CFGR_PPRE2_DIV2;    // APB2 = AHB / 2

    // 7. Switch system clock to PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait until PLL is system clock
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
