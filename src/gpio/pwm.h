#ifndef BUCKY_PWM_H
#define BUCKY_PWM_H

#include <Arduino.h>
#include <stm32g4xx.h>
#include <PeripheralPins.h>

struct PwmPin {
    TIM_TypeDef* timer;
    volatile uint32_t* ccr; // pointer to CCR1/CCR2/CCR3/CCR4
    uint8_t channel;        // 0-based channel index
    bool complementary;     // true for CHxN (inverted) outputs

    explicit PwmPin(const int pin) {
        const PinName pn = digitalPinToPinName(pin);
        const uint32_t func = pinmap_function(pn, PinMap_TIM);
        timer = static_cast<TIM_TypeDef*>(pinmap_peripheral(pn, PinMap_TIM));
        channel = STM_PIN_CHANNEL(func) - 1; // convert 1-based to 0-based
        complementary = STM_PIN_INVERTED(func);
        ccr = &timer->CCR1 + channel; // CCR1-CCR4 are contiguous 32-bit registers
    }

    constexpr PwmPin() : timer(nullptr), ccr(nullptr), channel(0), complementary(false) {}
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

inline void pwmInit(const PwmPin& pw, const int pin, uint32_t freq = 1000, uint32_t resolution = 255) {
    // Hold pin LOW as GPIO while configuring the timer
    const PinName pn = digitalPinToPinName(pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    // Configure timer before connecting GPIO to prevent glitches
    pwmEnableClock(pw.timer);

    const uint32_t timerClk = HAL_RCC_GetPCLK1Freq();
    pw.timer->PSC = (timerClk / (freq * (resolution + 1))) - 1;
    pw.timer->ARR = resolution;

    const uint8_t ch = pw.channel;
    if (ch < 2) {
        const uint8_t shift = ch * 8;
        pw.timer->CCMR1 = (pw.timer->CCMR1 & ~(0xFFU << shift))
                         | (0x68U << shift); // OC mode 1 (0x60) + preload enable (0x08)
    } else {
        const uint8_t shift = (ch - 2) * 8;
        pw.timer->CCMR2 = (pw.timer->CCMR2 & ~(0xFFU << shift))
                         | (0x68U << shift);
    }

    // Start with 0 duty
    *pw.ccr = 0;

    // Enable channel output: CCxE for normal, CCxNE for complementary
    if (pw.complementary) {
        pw.timer->CCER |= (1U << (ch * 4 + 2)); // CCxNE
    } else {
        pw.timer->CCER |= (1U << (ch * 4));      // CCxE
    }

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

    // Enable counter
    pw.timer->CR1 |= TIM_CR1_CEN;

    // NOW switch pin to timer alternate function (seamless LOW -> 0% duty)
    pinmap_pinout(pn, PinMap_TIM);
}

inline void pwmWrite(const PwmPin& pw, uint32_t value) {
    *pw.ccr = value;
}

#endif //BUCKY_PWM_H
