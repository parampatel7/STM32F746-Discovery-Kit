/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern struct netif gnetif;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
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
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_LWIP_Init();
  printf("LWIP initialized!\r\n");

  /* USER CODE BEGIN 2 */
//  if (netif_is_link_up(&gnetif))
//  {
//	  //Link is UP
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5,RESET);
//	  HAL_Delay(1000);
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5,SET);
//	  HAL_Delay(1000);
//  }
//  else
//  {
//      // Link is DOWN
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5,RESET);
//	  HAL_Delay(10000);
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5,SET);
//	  HAL_Delay(10000);
//  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 // printf("Entered While loop. \r\n");
	  ethernetif_input(&gnetif);
	  sys_check_timeouts();

	  // Also add periodic link check
	  static uint32_t link_timer = 0;
	  if (HAL_GetTick() - link_timer >= 100)
	  {
	      link_timer = HAL_GetTick();
	      ethernet_link_check_state(&gnetif);
	  }
/*
┌─────────────────────────────────────┐
│  Application (Your ping request)    │
├─────────────────────────────────────┤
│  LwIP Stack (Software)              │
│  - IP, ARP, ICMP processing         │
├─────────────────────────────────────┤
│  netif (Network Interface)          │
│  - Link status (up/down)            │
├─────────────────────────────────────┤
│  STM32 ETH Driver (HAL)             │
│  - DMA (moves packets to/from RAM)  │
│  - MAC (Ethernet controller)        │
├─────────────────────────────────────┤
│  PHY Chip (LAN8742)                 │
│  - Physical link negotiation        │
│  - Speed/Duplex detection           │
├─────────────────────────────────────┤
│  Physical Cable & Network           │
└─────────────────────────────────────┘
When MX_LWIP_Init() is called: Inside ethernetif_init() (called by netif_add):
// Simplified version of what happens:
static err_t ethernetif_init(struct netif *netif)
{
  // Initialize HAL ETH peripheral
  HAL_ETH_Init(&heth);  // ← Configures MAC, allocates DMA descriptors

  // Initialize PHY
  LAN8742_Init(&LAN8742);  // ← Resets PHY chip

  // Configure MAC filters, etc.

  // ⚠️ NOTE: HAL_ETH_Start() is NOT called here!
  // The DMA is NOT started yet!

  return ERR_OK;
}
```

### Critical Point After Init:
```
Status after MX_LWIP_Init():
┌─────────────────────────────────────┐
│ LwIP Stack        → ✅ Ready        │
│ netif             → ✅ UP           │
│ MAC (ETH)         → ⚠️  Configured  │
│ DMA               → ❌ NOT STARTED  │
│ PHY               → 🔄 Negotiating  │
│ Physical Link     → 🔄 Negotiating  │
└─────────────────────────────────────┘
The PHY needs 1-3 seconds to negotiate with your switch/router:

Auto-negotiation of speed (10/100 Mbps)
Auto-negotiation of duplex (Half/Full)
This happens in the BACKGROUND while your code runs



--->The Problem Without ethernet_link_check_state():
while (1)
{
    ethernetif_input(&gnetif);  // Try to receive packets

    // ❌ Missing: sys_check_timeouts()
    // ❌ Missing: ethernet_link_check_state()

    // Blink LED
    if (HAL_GetTick() - last >= 500)
    {
        last = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_5);
    }
}
```

### What Happens:
```
Time: 0ms
├─ MX_LWIP_Init() completes
├─ MAC is configured but DMA is NOT started
├─ PHY starts negotiation (takes 1-3 seconds)
└─ Main loop starts running

Time: 10ms
├─ ethernetif_input() called → Returns NULL (no DMA running!)
└─ Loop continues...

Time: 100ms
├─ ethernetif_input() called → Returns NULL
└─ Loop continues...

Time: 2000ms (2 seconds)
├─ PHY negotiation COMPLETE! Link is UP at 100Mbps Full Duplex
├─ But nobody told the MAC about it!
├─ DMA still NOT started!
└─ ethernetif_input() → Still returns NULL

Time: 5000ms
├─ You try to ping 10.4.90.100
├─ Ping packet arrives at PHY → ✅
├─ PHY forwards to MAC → ✅
├─ MAC has no DMA running → ❌ Packet DROPPED
└─ Result: "Destination Host Unreachable"

The DMA engine was never started, so packets arriving at the MAC have nowhere to go. They're immediately discarded.

--->  Why It Works in Debug Mode:
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_LWIP_Init();
    printf("LWIP initialized!\r\n");

    // ← YOU SET A BREAKPOINT HERE

    while (1)
    {
        ethernetif_input(&gnetif);
        // ...
    }
}
```

**What happens:**
```
Time: 0ms
├─ MX_LWIP_Init() completes
├─ DMA NOT started
├─ PHY starts negotiation
└─ Hit breakpoint → Code PAUSES

Time: 0ms - 3000ms (You're paused in debugger)
├─ PHY negotiation completes
├─ Link is UP
└─ (Code is still paused)

Time: 3000ms - You resume execution
├─ Now you call ethernetif_input()
├─ Inside ethernetif_input(), there's usually code like:
│  if (HAL_ETH_GetDMAStatus() shows link changed)
│     → Calls some recovery code
│     → Might implicitly start DMA
└─ OR: Debug probe tools interfere with hardware state
When you step through code slowly:

Each step takes 100ms - 1000ms
By the time you reach the main loop, 5-10 seconds have passed
Some implementations of ethernetif_input() check link status and auto-start DMA if they detect a link
The slow execution gives these recovery mechanisms time to work

---> What ethernet_link_check_state() Actually Does:
void ethernet_link_check_state(struct netif *netif)
{
  // 1. Ask PHY: "What's your link status?"
  PHYLinkState = LAN8742_GetLinkState(&LAN8742);

  // 2. Compare PHY status with software status

  // Case A: Software thinks link is UP, but PHY says DOWN
  if(netif_is_link_up(netif) && (PHYLinkState <= LAN8742_STATUS_LINK_DOWN))
  {
    HAL_ETH_Stop(&heth);        // Stop DMA
    netif_set_down(netif);      // Tell LwIP: interface down
    netif_set_link_down(netif); // Tell LwIP: physical link down
  }

  // Case B: Software thinks link is DOWN, but PHY says UP ⭐
  else if(!netif_is_link_up(netif) && (PHYLinkState > LAN8742_STATUS_LINK_DOWN))
  {
    // PHY negotiated! Get the results
    switch (PHYLinkState)
    {
    case LAN8742_STATUS_100MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    // ... other cases
    }

    if(linkchanged)
    {
      // ⭐ THIS IS THE CRITICAL SECTION ⭐

      // Step 1: Get current MAC config
      HAL_ETH_GetMACConfig(&heth, &MACConf);

      // Step 2: Update MAC to match PHY negotiation
      MACConf.DuplexMode = duplex;  // Match PHY
      MACConf.Speed = speed;         // Match PHY
      HAL_ETH_SetMACConfig(&heth, &MACConf);

      // Step 3: START THE DMA ENGINE! 🎉
      HAL_ETH_Start(&heth);

      // Step 4: Tell LwIP software that link is up
      netif_set_up(netif);
      netif_set_link_up(netif);
    }
  }
}
```

---

## Part 6: The Complete Timeline - Without vs With Fix

### ❌ WITHOUT `ethernet_link_check_state()`:
```
0ms:    Power on
10ms:   MX_LWIP_Init() → MAC configured, DMA NOT started
11ms:   PHY starts negotiation
12ms:   Enter main loop
13ms:   ethernetif_input() → NULL (no DMA)
100ms:  ethernetif_input() → NULL
500ms:  ethernetif_input() → NULL
2000ms: PHY negotiation DONE → Link UP at 100Mbps Full
2001ms: ethernetif_input() → NULL (DMA still not started!)
5000ms: PING arrives → MAC receives → DMA not running → DROPPED
        Result: "Destination Host Unreachable" ❌
```

### ✅ WITH `ethernet_link_check_state()`:
```
0ms:    Power on
10ms:   MX_LWIP_Init() → MAC configured, DMA NOT started
11ms:   PHY starts negotiation
12ms:   Enter main loop
13ms:   ethernetif_input() → NULL
14ms:   ethernet_link_check_state() → PHY still negotiating, nothing to do
100ms:  ethernet_link_check_state() → PHY still negotiating
500ms:  ethernet_link_check_state() → PHY still negotiating
2000ms: PHY negotiation DONE → Link UP at 100Mbps Full
2100ms: ethernet_link_check_state() called:
        ├─ Detects: netif thinks DOWN, but PHY is UP
        ├─ Reads PHY status: 100Mbps Full Duplex
        ├─ Configures MAC: 100Mbps Full Duplex
        ├─ ⭐ HAL_ETH_Start() → DMA STARTED! ⭐
        └─ netif_set_link_up() → Software knows link is UP
2101ms: ethernetif_input() → Now DMA is running, can receive!
5000ms: PING arrives → MAC receives → DMA moves to RAM → LwIP processes
        ARP: "Who has 10.4.90.100?" → "I do! MAC: xx:xx:xx..."
        ICMP: "Echo request" → "Echo reply"
        Result: "64 bytes from 10.4.90.100: time=1ms" ✅


----> Why it didn't work after flashing:

DMA was never started because HAL_ETH_Start() was never called
Even though PHY negotiated a link, the MAC couldn't receive packets
Packets arrived but were immediately discarded

Why it worked in debug:

Extra time allowed PHY negotiation to complete
Slow execution triggered timeout/recovery mechanisms
Debug tools may have interfered with hardware state

Why ethernet_link_check_state() fixed it:

Monitors PHY status every 100ms
Detects when negotiation completes
Calls HAL_ETH_Start() to start DMA
Synchronizes PHY ↔ MAC ↔ LwIP software states
*/

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
