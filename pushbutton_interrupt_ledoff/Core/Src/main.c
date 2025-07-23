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
/*when kept push button pressed, LED turns off*/
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

volatile uint8_t buttonstate = 0;
void EXTI15_10_IRQHandler(void)
{
	if(EXTI->PR & EXTI_PR_PR11){ //CHECKS WHETHER INTERRUPT TRIGGERED, PR REGISTER HAS PENDING INTERRUPT FLAGS FOR ALL EXTI LINES
	EXTI->PR = EXTI_PR_PR11;}//CLEAR INTERRUPT FLAG
	if (GPIOI->IDR & (1 << 11))  // Rising edge: button pressed
	        {
	            GPIOD->ODR |= (1 << 5);  // Turn ON LED
	        }
	        else                         // Falling edge: button released
	        {
	            GPIOD->ODR &= ~(1 << 5); // Turn OFF LED
	        }
}


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


  while (1)
  {
    /* USER CODE END WHILE */

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

	//CONFIGURE EXTI LINE 11 FOR PI11
	/*SYSCFG_EXTICR : These bits are written by software to select the source input for the EXTIx
	external interrupt.
	0000: PA[x] pin
0001: PB[x] pin
0010: PC[x] pin
0011: PD[x] pin
0100: PE[x] pin
0101: PF[x] pin
0110: PG[x] pin
0111: PH[x] pin
1000: PI[x] pin
1001:PJ[x] pin
1010:PK[x] pin
EXTICR[2] manages lines 8–11*/

/*EXTICR[0]	0 to 3
EXTICR[1]	4 to 7
EXTICR[2]	8 to 11 <---- Line 11 here
EXTICR[3]	12 to 15*/

	//-------------------------didn't understand this below -------------------------------------------------------------------------------------
	SYSCFG->EXTICR[2] &= ~(0xF << 12);   // Clear EXTI11 bits, OXF IS GOOD PRACTISE TO CLEAR BITS
    SYSCFG->EXTICR[2] |=  (0x8 << 12);   // Set EXTI11 to Port I, 0X8 IS GOOD PRACTISE TO SET BITS AS ALL BITS NEED TO BE OPERATED TO PREVENT MALFUNCITON

    EXTI->IMR  |= (1 << 11);             // Interrupt request from line 11 is not masked, ALLOWING IT TO GENERATE INTERRUPT, IF NOT DONE NOTHING TRIGGERS
    EXTI->RTSR |= (1 << 11);             // Rising trigger enabled (for Event and Interrupt) for input line
    EXTI->FTSR |= (1 << 11);			// Falling edge trigger enabled
    // Enable NVIC for EXTI15_10
    NVIC_EnableIRQ(EXTI15_10_IRQn);  //Activates the interrupt handler for lines 10–15
    NVIC_SetPriority(EXTI15_10_IRQn, 1); //Sets the priority level. Lower values mean higher priority (0 is the highest).
/*PRIORITY CAN ASLO BE ASSIGNED 0, BUT ITS TYPICALLY RESERVED FOR SYSTEM CRITICAL FUNCTIONS LIKE HARD FAULTS/ TIMER OVERFLOWS THAT MUST PREEMPT EVERYTHING
 * ALSO CAN BLOCK OTHER LESS PRIORITY TASKS LIKE SysTick OR OS SCHEDULING*/
}
/**
  * @brief System Clock Configuration
  * @retval None
  */



void SystemClock_Config(void) {
    // 1. Enable HSI
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));  // Wait until HSI is ready

    // 2. Enable Power interface clock and configure OverDrive
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    PWR->CR1 |= PWR_CR1_ODEN;  // Enable OverDrive
    while (!(PWR->CSR1 & PWR_CSR1_ODRDY)); // Wait until OverDrive is ready

    PWR->CR1 |= PWR_CR1_ODSWEN;  // Start OverDrive switch
    while (!(PWR->CSR1 & PWR_CSR1_ODSWRDY)); // Wait until switch is ready

    // 3. Configure Flash latency and cache settings
    FLASH->ACR = FLASH_ACR_LATENCY_7WS |
                 FLASH_ACR_PRFTEN |
                 FLASH_ACR_ARTEN;

    // 4. Configure PLL to use HSI (16 MHz)
    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos)  | // PLLM = 16
                   (432 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 432
                   (0 << RCC_PLLCFGR_PLLP_Pos)   | // PLLP = 2
                   (RCC_PLLCFGR_PLLSRC_HSI)      | // PLL source = HSI
                   (9 << RCC_PLLCFGR_PLLQ_Pos);    // PLLQ = 9

    // 5. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));  // Wait for PLL to lock

    // 6. Set clock prescalers
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
