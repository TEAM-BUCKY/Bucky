#ifndef BUCKY_PWM_H
#define BUCKY_PWM_H

#include <Arduino.h>
#include <stm32g4xx.h>
#include <PeripheralPins.h>

struct PwmPin {
    TIM_TypeDef* timer;
    volatile uint32_t* ccr; // pointer to CCR1/CCR2/CCR3/CCR4
    uint8_t channel;        // 0-based channel index

    explicit PwmPin(const int pin) {
        const PinName pn = digitalPinToPinName(pin);
        timer = static_cast<TIM_TypeDef*>(pinmap_peripheral(pn, PinMap_TIM));
        channel = STM_PIN_CHANNEL(pinmap_function(pn, PinMap_TIM));
        ccr = &timer->CCR1 + channel; // CCR1-CCR4 are contiguous 32-bit registers
    }

    constexpr PwmPin() : timer(nullptr), ccr(nullptr), channel(0) {}
};

// Enable the timer's peripheral clock
inline void pwmEnableClock(const TIM_TypeDef* tim) {
    // APB1 timers
    if (tim == TIM2)       RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    else if (tim == TIM3)  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    else if (tim == TIM4)  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
#ifdef TIM5
    else if (tim == TIM5)  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
#endif
#ifdef TIM6
    else if (tim == TIM6)  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
#endif
#ifdef TIM7
    else if (tim == TIM7)  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
#endif
    // APB2 timers
    else if (tim == TIM1)  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
#ifdef TIM8
    else if (tim == TIM8)  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
#endif
#ifdef TIM15
    else if (tim == TIM15) RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
#endif
#ifdef TIM16
    else if (tim == TIM16) RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
#endif
#ifdef TIM17
    else if (tim == TIM17) RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
#endif
#ifdef TIM20
    else if (tim == TIM20) RCC->APB2ENR |= RCC_APB2ENR_TIM20EN;
#endif
}

// Initialize a pin for PWM output. Call once during setup.
// freq: PWM frequency in Hz, resolution: counter period (e.g., 255 for 8-bit)
inline void pwmInit(const PwmPin& pw, const int pin, uint32_t freq = 1000, uint32_t resolution = 255) {
    // Configure GPIO alternate function
    const PinName pn = digitalPinToPinName(pin);
    pinmap_pinout(pn, PinMap_TIM);

    // Enable timer clock
    pwmEnableClock(pw.timer);

    // Set prescaler: timer_clk / (freq * resolution)
    const uint32_t timerClk = HAL_RCC_GetPCLK1Freq(); // timers run at PCLK (or 2x if APB prescaler != 1)
    pw.timer->PSC = (timerClk / (freq * (resolution + 1))) - 1;
    pw.timer->ARR = resolution;

    // Configure channel for PWM mode 1 (active while CNT < CCR)
    const uint8_t ch = pw.channel;
    if (ch < 2) {
        // Channels 0,1 use CCMR1
        const uint8_t shift = ch * 8;
        pw.timer->CCMR1 = (pw.timer->CCMR1 & ~(0xFFU << shift))
                         | (0x68U << shift); // OC mode 1 (0x60) + preload enable (0x08)
    } else {
        // Channels 2,3 use CCMR2
        const uint8_t shift = (ch - 2) * 8;
        pw.timer->CCMR2 = (pw.timer->CCMR2 & ~(0xFFU << shift))
                         | (0x68U << shift);
    }

    // Enable channel output
    pw.timer->CCER |= (1U << (ch * 4));

    // For advanced timers (TIM1, TIM8, TIM20): enable main output
    if (pw.timer == TIM1
#ifdef TIM8
        || pw.timer == TIM8
#endif
#ifdef TIM20
        || pw.timer == TIM20
#endif
    ) {
        pw.timer->BDTR |= TIM_BDTR_MOE;
    }

    // Start with 0 duty
    *pw.ccr = 0;

    // Enable counter
    pw.timer->CR1 |= TIM_CR1_CEN;
}

// Set duty cycle - this is the hot path, single register write
inline void pwmWrite(const PwmPin& pw, uint32_t value) {
    *pw.ccr = value;
}

#endif //BUCKY_PWM_H
