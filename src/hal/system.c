#include "system.h"

static volatile uint32_t tick_ms;

static void clock_init(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    __DSB();
    PWR->CR5 &= ~PWR_CR5_R1MODE;

    FLASH->ACR = (4U << FLASH_ACR_LATENCY_Pos)
               | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != (4U << FLASH_ACR_LATENCY_Pos));

    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY);

    RCC->PLLCFGR = (3U  << RCC_PLLCFGR_PLLM_Pos)   /* M = 4  (reg = M-1) */
                 | (85U << RCC_PLLCFGR_PLLN_Pos)     /* N = 85             */
                 | (0U  << RCC_PLLCFGR_PLLR_Pos)     /* R = 2  (0->2)      */
                 | RCC_PLLCFGR_PLLREN                 /* Enable PLLR output */
                 | (2U  << RCC_PLLCFGR_PLLSRC_Pos);  /* Source = HSI16     */

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (3U << RCC_CFGR_SW_Pos);
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != (3U << RCC_CFGR_SWS_Pos));

    SystemCoreClock = SYS_CLOCK_HZ;
}

void system_init(void) {
    NVIC_SetPriorityGrouping(3);

    clock_init();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    SysTick_Config(SYS_CLOCK_HZ / 1000);

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    __DSB();
}

void SysTick_Handler(void) {
    tick_ms++;
}

uint32_t millis(void) {
    return tick_ms;
}

uint32_t micros(void) {
    __disable_irq();
    uint32_t ms = tick_ms;
    uint32_t val = SysTick->VAL;
    /* If SysTick has wrapped but ISR hasn't fired yet, account for it */
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {
        ms++;
        val = SysTick->VAL;
    }
    __enable_irq();
    return ms * 1000U + (SysTick->LOAD + 1U - val) / (SYS_CLOCK_HZ / 1000000U);
}

void delay(const uint32_t ms) {
    const uint32_t start = millis();
    while (millis() - start < ms);
}

void delay_us(const uint32_t us) {
    const uint32_t start = DWT->CYCCNT;
    const uint32_t ticks = us * (SYS_CLOCK_HZ / 1000000U);
    while (DWT->CYCCNT - start < ticks);
}
