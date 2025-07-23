#include "stm32f7xx.h"
#include "main.h"
/*software interrupt code to blink LED*/
/*SYSCFG_EXTICR not used as its for input interrupt, however for software interrupt there is no physical pin required*/

// Forward declarations
void init_gpio(void);
void init_exti11(void);
void trigger_software_interrupt(void);
void delay(volatile uint32_t count);

// Main
int main(void) {
    init_gpio();
    init_exti11();

    while (1) {
        delay(5000000);
        trigger_software_interrupt();  // trigger the LED toggle interrupt
    }
}

// Delay loop to visually see LED blinking
void delay(volatile uint32_t count) {
    while (count--) ;
}

// Setup PD5 as output
void init_gpio(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;       // Enable clock for GPIOD
    while (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIODEN)); // Wait clock stable

    GPIOD->MODER &= ~(3U << (5 * 2));  // Clear mode bits
    GPIOD->MODER |=  (1U << (5 * 2));  // Set PD5 to push-pull output
    GPIOD->OTYPER &= ~(1U << 5);
    GPIOD->OSPEEDR |= (3U << (5 * 2)); // High speed
    GPIOD->PUPDR &= ~(3U << (5 * 2));  // No pull-up/down
}

// EXTI line setup for software trigger
void init_exti11(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;     // Enable SYSCFG clock

    while (!(RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN));//system configuration clock enable

    EXTI->IMR |= (1U << 11);   // Unmask EXTI line 11
    EXTI->RTSR |= (1U << 11);  // Enable rising edge trigger

    NVIC_SetPriority(EXTI15_10_IRQn, 1);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// Manually fire EXTI11 interrupt
void trigger_software_interrupt(void) {//this function is called, it calls interrupt which is executed from ISR
    EXTI->SWIER |= (1U << 11);  //software interrupt register, Why pin 11: push button sample
}/*Equivalent to mcu thinking rising edge occurred at pin PI11 even if no actual signal occurred*/

// ISR for EXTI lines 10–15
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1U << 11)) {
        EXTI->PR |= (1U << 11);     // Clear pending flag
        GPIOD->ODR ^= (1U << 5);    // Toggle LED on PD5
    }
}
