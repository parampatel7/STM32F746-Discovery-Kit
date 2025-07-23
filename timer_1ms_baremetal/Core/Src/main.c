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
/*Timer for 1ms bare metal code to generate LED blink at 5s*/
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
void gpio_init(void);

/* USER CODE END PFP */
void TIM6_INIT(void);
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
SystemClock_Config();
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
 gpio_init();
 TIM6_INIT();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  GPIOD->ODR ^= (1 <<5);
	  	for(uint32_t i=0; i<5000U ; i++)
	  	{
	  		while((TIM6->SR & TIM_SR_UIF) == 0);
	  		TIM6->SR &= ~TIM_SR_UIF; //CLEAR UPDATE FLAG
	  	}


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void TIM6_INIT()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; //ENABLES TIMER6 CLOCK
	//SET PRESCALER AND AUTORELOAD FOR 1MS
	//ASSUMING APB1 TIMER CLOCK = 108MHZ (FROM 216MHZ SYSTEM CLOCK)
	//TO GET 1MS TICK: (PSC + 1) * (ARR + 1) = 108000
	// PSC=10799, ARR=9

	TIM6->PSC= 10799U;  //PRESCALER  108000
	TIM6->ARR = 9U;     //AUTORELOAD
	TIM6->EGR = TIM_EGR_UG; // force update to load PSC/ARR
	TIM6->SR = 0;           // clear flags
	TIM6->CR1 |= TIM_CR1_ARPE; // optional preload
	TIM6->CR1 |= TIM_CR1_CEN;		//ENABLE COUNTER

}
void gpio_init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
		GPIOD->MODER &= ~(3U << 10);
		GPIOD->MODER |= (1U << 10);

		GPIOD->OSPEEDR &= ~(3U << 10);
		GPIOD->OSPEEDR |= (1U << 10);
		GPIOD->PUPDR |= (1U << 22);

}

void SystemClock_Config(void)
{
	RCC->CR |= RCC_CR_HSEON ;
		  while(!(RCC->CR & RCC_CR_HSERDY));

		  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
		  PWR-> CR1 |= PWR_CR1_ODEN;
		  while(!(PWR->CSR1 & PWR_CSR1_ODRDY));

		  PWR->CR1 |= PWR_CR1_ODSWEN;
		  while(!(PWR->CSR1 & PWR_CSR1_ODSWRDY));

		  FLASH->ACR = FLASH_ACR_LATENCY_7WS | FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;
//HSE=25MHz SYSCLK WE NEED 216MHz
		  RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos) |
				  (432 << RCC_PLLCFGR_PLLN_Pos) |
				   (0 << RCC_PLLCFGR_PLLP_Pos)   |//issue caused by this line
/*      PLLP = 2 is encoded as 00 (i.e., 0)
        PLLP = 4 → 01 (1)
        PLLP = 6 → 10 (2)
        PLLP = 8 → 11 (3)
 * */
				                     (RCC_PLLCFGR_PLLSRC_HSE)      |
				                     (9 << RCC_PLLCFGR_PLLQ_Pos);    // PLLQ = 9

		  RCC-> CR |= RCC_CR_PLLON;
		  while(!(RCC->CR & RCC_CR_PLLRDY));

		  RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

		  RCC->CFGR &= ~RCC_CFGR_SW;
		  RCC->CFGR |= RCC_CFGR_SW_PLL;
		  while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
//using HSI clock
/*void SystemClock_Config(void)
{
    // Enable HSI
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0);

    // Enable power interface clock and voltage scaling
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR1 |= PWR_CR1_ODEN;
    while (!(PWR->CSR1 & PWR_CSR1_ODRDY));

    PWR->CR1 |= PWR_CR1_ODSWEN;
    while (!(PWR->CSR1 & PWR_CSR1_ODSWRDY));

    // Set Flash latency and enable caches
    FLASH->ACR = FLASH_ACR_LATENCY_7WS | FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;

    // Configure PLL for HSI = 16 MHz input → 216 MHz SYSCLK
    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos) |   //check this as I think I have changed it unknowingly
                   (432 << RCC_PLLCFGR_PLLN_Pos) |
                   (2 << RCC_PLLCFGR_PLLP_Pos) |      // PLLP = 2 (00)
                   (RCC_PLLCFGR_PLLSRC_HSI) |         // PLL source = HSI
                   (9 << RCC_PLLCFGR_PLLQ_Pos);       // PLLQ = 9 for USB etc.

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);

    // Configure AHB, APB1, APB2 prescalers
    RCC->CFGR = RCC_CFGR_HPRE_DIV1  |   // AHB = SYSCLK / 1 = 216 MHz
                RCC_CFGR_PPRE1_DIV4 |   // APB1 = 54 MHz
                RCC_CFGR_PPRE2_DIV2;    // APB2 = 108 MHz

    // Switch system clock to PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
*/

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
