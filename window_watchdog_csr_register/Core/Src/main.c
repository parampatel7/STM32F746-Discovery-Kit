#include "stm32f7xx.h"
#include "main.h"

#define LED_OFF 								GPIOD-> ODR |=(1<<5);
#define LED_ON  								GPIOD->ODR &= ~(1<<5);
#define RESET_Watchdog 							WWDG_CR_WDGA | (0x7F)
#define RCC_CLOCK_ON 							APB1ENR |= RCC_APB1ENR_WWDGEN
#define SET_WINDOW_PRESCALER					CFR =  (3U << WWDG_CFR_WDGTB_Pos)| (80U << WWDG_CFR_W_Pos)
#define CLOCK_TO_GPIO							AHB1ENR |= RCC_AHB1ENR_GPIODEN
#define CLEAR_MODE_BIT							MODER &= ~(0x3 << (5 * 2))
#define SET_MODE_BIT							MODER |=  (0x1 << (5 * 2))
#define SET_TYPE								OTYPER &= ~(1 << 5)
#define SET_SPEED								OSPEEDR &= ~(0x3 << (5 * 2))
#define SET_PULLUP_PULLDOWN						PUPDR &= ~(0x3 << (5 * 2))
#define CLEAR_RESET_FLAG						RCC->CSR |= RCC_CSR_RMVF
#define POWER_RESET								RCC->CSR & RCC_CSR_PORRSTF
#define WINDOW_WATCHDOG_RESET					RCC->CSR & RCC_CSR_WWDGRSTF

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void wwdg_init(void);
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
  HAL_Init();
  SystemClock_Config();
  wwdg_init();
  MX_GPIO_Init();
//CLEAR_RESET_FLAG

  if(POWER_RESET)
  {
	  for(int i=0; i<5; i++)
	  {	  LED_ON;
	  	  HAL_Delay(1000);
	  	  LED_OFF;
	  	  HAL_Delay(1000);
	  }

  }
  if(WINDOW_WATCHDOG_RESET )
  	  {
  		  for(int i=0; i<5; i++)
  		  {	  LED_ON;
  		  	  HAL_Delay(5000);
  		  	  LED_OFF;
  		  	  HAL_Delay(1000);
  		  }
  	  }
for(int j=0; j < 204; j++)
  {
	  // Must refresh after ~28.5 ms but before ~38 ms
	  HAL_Delay(30);  // Safe refresh timing
	  WWDG->CR = RESET_Watchdog;
	  LED_ON;
  }


 // CLEAR_RESET_FLAG;


//  for(int j=0; j < 204; j++)
//          {
//              // Must refresh after ~28.5 ms but before ~38 ms
//              HAL_Delay(30);  // Safe refresh timing
//              WWDG->CR = RESET_Watchdog;
//              LED_ON;
//          }


  while (1)
  {

	    /* USER CODE END WHILE */
//		    for(int j=0 ; j <5; j++)
//		    {
//		  	LED_ON;
//		  	HAL_Delay(2000);
//		  	LED_OFF;
//		  	HAL_Delay(2000);
//		    }


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
  RCC_OscInitStruct.PLL.PLLN = 432;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
	RCC-> CLOCK_TO_GPIO;
	    GPIOD->CLEAR_MODE_BIT;
	    GPIOD->SET_MODE_BIT;
	    GPIOD->SET_TYPE;
	    GPIOD->SET_SPEED;
	    GPIOD->SET_PULLUP_PULLDOWN;
	    LED_OFF;
}
void wwdg_init(void)
{
	RCC-> RCC_CLOCK_ON;          // Enable WWDG clock as wwdt needs PCLK

	    /* Configure CFR: WDGTB=
	     *  00: CK counter clock (PCLK div 4096) div 1
			01: CK counter clock (PCLK div 4096) div 2
			10: CK counter clock (PCLK div 4096) div 4
			11: CK counter clock (PCLK div 4096) div 8 */
	 WWDG-> SET_WINDOW_PRESCALER;           // window = 80
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
