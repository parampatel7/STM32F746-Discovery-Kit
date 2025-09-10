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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
 * ---------------------------------------------------------------------------------------------------
 * If code debugging halts at Systemclock_config even if code is auto generated, it might be faulty
 * as for that particular frequency those solutions might not be valid even if they are configured automatically
 * and no "Resolve clock error" is shown.
 *
 * IN SPI FOR FULL DUPLEX MODE, TRANSMITTING ONE BYTE WILL LEAD TO RECEIVING ONE BYTE
 * ----------------------------------------------------------------------------------------------------*/
#include "stm32f7xx.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint16_t read_value = 0;
uint16_t debug_buffer = 0;
uint8_t rx1 = 0, rx2 = 0;
uint8_t target_register=0x91;
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
void spi2_init(void);
void AD7779_ReadRegister(uint8_t reg);
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
  gpio_init();
  spi2_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  AD7779_ReadRegister(target_register);
  read_value = debug_buffer;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

	  /** Configure the main internal regulator output voltage
	  */
	  __HAL_RCC_PWR_CLK_ENABLE();
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/* USER CODE BEGIN 4 */
void gpio_init(void)
{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // Enable clock for GPIOB
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // Enable clock for GPIOD

	    // 2. Set MODER to Alternate Function (10) for each pin
	    // PD3
	    GPIOD->MODER &= ~(0x3 << (3 * 2));      // Clear mode bits
	    GPIOD->MODER |=  (0x2 << (3 * 2));      // Set to AF mode

	    // PB12, PB14, PB15
	    GPIOB->MODER &= ~((0x3 << (12 * 2)) | (0x3 << (14 * 2)) | (0x3 << (15 * 2)));
	    GPIOB->MODER |=  ((0x2 << (12 * 2)) | (0x2 << (14 * 2)) | (0x2 << (15 * 2)));

	    // 3. Set Alternate Function to AF5 (SPI2) for each pin
	    // AF registers: AFR[0] for pins 0-7, AFR[1] for pins 8-15

	    // PD3 -> AF5
	    GPIOD->AFR[0] &= ~(0xF << (3 * 4));
	    GPIOD->AFR[0] |=  (0x5 << (3 * 4));

	    // PB12, PB14, PB15 -> AF5
	    GPIOB->AFR[1] &= ~((0xF << ((12 - 8) * 4)) | (0xF << ((14 - 8) * 4)) | (0xF << ((15 - 8) * 4)));
	    GPIOB->AFR[1] |=  ((0x5 << ((12 - 8) * 4)) | (0x5 << ((14 - 8) * 4)) | (0x5 << ((15 - 8) * 4)));

	    // 4. Set output type to Push-Pull (0)
	    GPIOD->OTYPER &= ~(1 << 3);
	    GPIOB->OTYPER &= ~((1 << 12) | (1 << 14) | (1 << 15));

	    // 5. Set speed to Very High (11) – optional but recommended for SPI
	    GPIOD->OSPEEDR |= (0x3 << (3 * 2));
	    GPIOB->OSPEEDR |= (0x3 << (12 * 2)) | (0x3 << (14 * 2)) | (0x3 << (15 * 2));

	    // 6. Set to No Pull-up/Pull-down (00)
	    GPIOD->PUPDR &= ~(0x3 << (3 * 2));
	    GPIOB->PUPDR &= ~((0x3 << (12 * 2)) | (0x3 << (14 * 2)) | (0x3 << (15 * 2)));

}
void spi2_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	SPI2->CR1 &= ~SPI_CR1_SPE;      // Disable before config
	SPI2->CR1 &= ~(SPI_CR1_BIDIMODE);
	SPI2->CR1 &= ~SPI_CR1_CRCEN;
	SPI2->CR1 &= ~(7U << 3);//SET BAUD RATE PRESCALER BY 2
	SPI2->CR1 |= SPI_CR1_MSTR; //SET AS MASTER
	SPI2->CR1 &= ~SPI_CR1_LSBFIRST;
	SPI2->CR1 |= SPI_CR1_CPOL;
	SPI2->CR1 |= SPI_CR1_CPHA;
	SPI2->CR2 &= ~SPI_CR2_DS; // Clear bits
	SPI2->CR2 |= (SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0); // 8-bit mode
	SPI2->CR2 |= SPI_CR2_SSOE;
	SPI2->CR2 &= ~SPI_CR2_NSSP;
	//SPI2->CR2 &= ~SPI_CR2_FRXTH;    // Optional: FIFO threshold = 8-bit
	//SPI2->CR2 |= SPI_CR2_NSSP;
	SPI2->CR1 |=	SPI_CR1_SPE;  //ENABLE SPI2
}

void AD7779_ReadRegister(uint8_t reg)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
//
    tx_buf[0] = reg;   // reg = 0x99 (already has read bit set)
    tx_buf[1] = 0x00;  // dummy byte to receive data
//
//    // Full-duplex SPI: transmit 2 bytes, receive 2 bytes
//    //HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 2, HAL_MAX_DELAY);
//    //ADDRESS TRANSMIT FIRST, THEN DUMMY BYTE, THEN RECEIVE DATA
//    SPI2->DR = tx_buf[0];
//    while(!(SPI2->SR & SPI_SR_TXE)); //WAIT FOR TRANSMISSION TO BE COMPLETED
//    SPI2->DR = tx_buf[1];
//    while(!(SPI2->SR & SPI_SR_TXE));
//    while(!(SPI2->SR & SPI_SR_RXNE)); //WAIT FOR DATA TO BE RECEIVED
//    SPI2->DR &= ~(0xFF00 << 0); // GET THE LOWER 8BITS HAVING DATA
//    rx_buf = SPI2->DR;
//    // Register data is returned in second byte
//    debug_buffer = rx_buf[1];
	  GPIOB->ODR &= ~(1 << 12); // NSS low

	    // Send register address
	    while (!(SPI2->SR & SPI_SR_TXE));
	   // SPI2->DR = reg;
	    *((__IO uint8_t*)&SPI2->DR) = tx_buf[0];  // Send first byte (reg)

	    while (!(SPI2->SR & SPI_SR_RXNE)); // Wait for dummy response
	    //uint8_t dummy = SPI2->DR;
	    rx_buf[0] = *((__IO uint8_t*)&SPI2->DR);  // Read and discard first byte (dummy read)


	    // Send dummy byte to receive actual data
	    while (!(SPI2->SR & SPI_SR_TXE));
	    //SPI2->DR = 0x00;
	    *((__IO uint8_t*)&SPI2->DR) = tx_buf[1];  // Send dummy byte


	    while (!(SPI2->SR & SPI_SR_RXNE)); // Wait for actual data
	    //rx2 = SPI2->DR;
	    rx_buf[1] = *((__IO uint8_t*)&SPI2->DR);  // Read received register value
	    //GPIOB->ODR |= (1 << 12); // NSS high

	    debug_buffer = rx_buf[2];

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
