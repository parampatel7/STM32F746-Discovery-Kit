
/*General purpose timer 2 configured for 5s delay*/
/*The pre and arr values are retained in hardware registers, we do not need to reload each time we enable/disable timer*/
/*for interrupt enable interrupt from DIER Register*/
/*There are many pwm modes: edge aligned, centre aligned, asymmetric, combined
 *Mode	         Counter Type	Pulse Shape	             Usage
Edge-aligned	Up-count only	One-sided       Simple PWM, LED dimming, fans
Center-aligned	Up-down count	Symmetric	    Motor control, inverter, less harmonics
Asymmetric	    Up-down count	Skewed shape	Resonant converters, complex switching
Combined PWM	Advanced only	Complementary	Half/full bridge control, 3-phase motors*/

/*For PWM changes made: GPIO Alternate function
 * AFR is split into 2 AFRL for GPIO 0 to 7, AFRH for GPIO 8 to 15*/

/*No timer channel connected to PD5, can check from data sheet -> pin configuration*/
/*One solution: Configure two timers, for ON and OFF*/



#include "stm32f7xx.h"

void GPIO_Init(void);
void TIM2_Init(void);
void TIM3_Init(void);
void SystemClock_Config(void);
int main(void)
{
	SystemClock_Config();
    GPIO_Init();
    TIM2_Init();
    TIM3_Init();

    TIM2->CR1 |= TIM_CR1_CEN; // Start TIM2 to begin the blink cycle

    while (1) {
        // LED control handled by interrupts
    }
}

void GPIO_Init(void)
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
void TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 10799;
    TIM2->ARR = 19999; // ≈2 seconds
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_OPM; // One pulse mode
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);
}

void TIM3_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 10799;
    TIM3->ARR = 19999; // ≈2 seconds
    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_OPM; // One pulse mode
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 1);
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        GPIOD->ODR |= (1UL << 5);   // LED ON
        TIM3->CR1 |= TIM_CR1_CEN;   // Start OFF timer
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;
        GPIOD->ODR &= ~(1UL << 5);  // LED OFF
        TIM2->CR1 |= TIM_CR1_CEN;   // Start ON timer
    }
}

