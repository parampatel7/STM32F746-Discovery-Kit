#include "main.h"
#include "stm32f7xx.h"

// --- Global variables ---
uint8_t buttonstate = 1;
uint8_t lastbuttonstate = 1;
uint8_t ledstate = 0;

// --- Function prototypes ---
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int check_rising_edge(void);
void buttonstate_presentstate_to_laststate(void);
void toggle_LED_state(void);
void output_to_LEDPin(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  while (1)
  {
    if (check_rising_edge())  // Detect falling edge
    {
      toggle_LED_state();
      output_to_LEDPin();
      HAL_Delay(200);  // Debounce delay
    }

    buttonstate_presentstate_to_laststate();
  }
}

// --- Function Definitions ---

int check_rising_edge(void)
{
  buttonstate = (GPIOI->IDR & (1U << 11)) ? 1 : 0;

  if (buttonstate == 0 && lastbuttonstate == 1)
    return 1;
  else
    return 0;
}

void buttonstate_presentstate_to_laststate(void)
{
  lastbuttonstate = buttonstate;
}

void toggle_LED_state(void)
{
  ledstate = !ledstate;
}

void output_to_LEDPin(void)
{
  if (ledstate)  // LED active LOW
    GPIOD->ODR &= ~(1U << 5);  // Turn ON LED
  else
    GPIOD->ODR |= (1U << 5);   // Turn OFF LED
}

static void MX_GPIO_Init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOIEN;

  // PD5 as output
  GPIOD->MODER &= ~(3U << 10);
  GPIOD->MODER |= (1U << 10);
  GPIOD->OSPEEDR &= ~(3U << 10);
  GPIOD->OSPEEDR |= (1U << 10);
  GPIOD->PUPDR &= ~(3U << 10);

  // PI11 as input with pull-up
  GPIOI->MODER &= ~(3U << 22);
  GPIOI->PUPDR &= ~(3U << 22);
  GPIOI->PUPDR |= (1U << 22);
}



void SystemClock_Config(void) {
    // 1. Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));  // Wait until HSE is ready

    // 2. Configure Power Regulator
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR1 |= PWR_CR1_ODEN;  // Overdrive enable
    while (!(PWR->CSR1 & PWR_CSR1_ODRDY));

    PWR->CR1 |= PWR_CR1_ODSWEN;  // Switch Overdrive
    while (!(PWR->CSR1 & PWR_CSR1_ODSWRDY));

    // 3. Configure Flash latency
    FLASH->ACR = FLASH_ACR_LATENCY_7WS | FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;

    // 4. Configure PLL
    RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos)  | // PLLM = 25
                   (432 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 432
                   (0 << RCC_PLLCFGR_PLLP_Pos)   | // PLLP = 2 (00 = /2)
                   (RCC_PLLCFGR_PLLSRC_HSE)      |
                   (9 << RCC_PLLCFGR_PLLQ_Pos);    // PLLQ = 9

    // 5. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));  // Wait for PLL to lock

    // 6. Configure clock dividers
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 |    // AHB = SYSCLK / 1
                RCC_CFGR_PPRE1_DIV4 |   // APB1 = AHB / 4
                RCC_CFGR_PPRE2_DIV2;    // APB2 = AHB / 2

    // 7. Switch system clock to PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait until PLL is system clock
}


void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
