#ifndef BUCKY_PWM_H
#define BUCKY_PWM_H

#include <Arduino.h>
#include <stm32g4xx.h>
#include <PeripheralPins.h>

#include "bitboard/bitboard.h"

template <bool enable>
constexpr void toggleTimer(TIM_TypeDef* tim)
{
    if constexpr (enable) setMask(tim->CR1, TIM_CR1_CEN);
    else clearMask(tim->CR1, TIM_CR1_CEN);
}

struct PwmPin {
    TIM_TypeDef* timer;
    volatile uint32_t* ccr; // pointer to CCR1/CCR2/CCR3/CCR4
    uint8_t channel;        // 0-based channel index
    uint8_t syncIndex;      // index into PwmSyncState::pins (0xFF = unregistered)
    bool complementary;     // true for CHxN (inverted) outputs

    explicit PwmPin(const int pin) {
        const PinName pn = digitalPinToPinName(pin);
        const uint32_t func = pinmap_function(pn, PinMap_TIM);
        timer = static_cast<TIM_TypeDef*>(pinmap_peripheral(pn, PinMap_TIM));
        channel = STM_PIN_CHANNEL(func) - 1; // convert 1-based to 0-based
        complementary = STM_PIN_INVERTED(func);
        ccr = &timer->CCR1 + channel; // CCR1-CCR4 are contiguous 32-bit registers
        syncIndex = 0xFF;
    }

    constexpr PwmPin() : timer(nullptr), ccr(nullptr), channel(0), syncIndex(0xFF), complementary(false) {}
};

inline void pwmEnableClock(const TIM_TypeDef* tim) {
    switch (reinterpret_cast<uint32_t>(tim)) {
        case TIM2_BASE:  setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM2EN);  break;
        case TIM3_BASE:  setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM3EN);  break;
        case TIM4_BASE:  setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM4EN);  break;
#ifdef TIM5_BASE
        case TIM5_BASE:  setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM5EN);  break;
#endif
#ifdef TIM6_BASE
        case TIM6_BASE:  setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM6EN);  break;
#endif
#ifdef TIM7_BASE
        case TIM7_BASE:  setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM7EN);  break;
#endif
        // APB2 timers
        case TIM1_BASE:  setMask(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);    break;
#ifdef TIM8_BASE
        case TIM8_BASE:  setMask(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);    break;
#endif
#ifdef TIM15_BASE
        case TIM15_BASE: setMask(RCC->APB2ENR, RCC_APB2ENR_TIM15EN);   break;
#endif
#ifdef TIM16_BASE
        case TIM16_BASE: setMask(RCC->APB2ENR, RCC_APB2ENR_TIM16EN);   break;
#endif
#ifdef TIM17_BASE
        case TIM17_BASE: setMask(RCC->APB2ENR, RCC_APB2ENR_TIM17EN);   break;
#endif
#ifdef TIM20_BASE
        case TIM20_BASE: setMask(RCC->APB2ENR, RCC_APB2ENR_TIM20EN);   break;
#endif
        default: break;
    }
}

inline void pwmInit(const PwmPin& pw, const int pin, const uint32_t freq = 1000, const uint32_t resolution = 255) {
    const PinName pn = digitalPinToPinName(pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    pwmEnableClock(pw.timer);

    const uint32_t timerClk = HAL_RCC_GetPCLK1Freq();
    pw.timer->PSC = timerClk / (freq * (resolution + 1)) - 1;
    pw.timer->ARR = resolution;

    const uint8_t ch = pw.channel;

    volatile uint32_t* ccmr = &pw.timer->CCMR1 + (ch >> 1);
    const uint8_t shift = (ch & 1) * 8;
    writeField(*ccmr, 0xFFU, shift, 0x68U); // OC mode 1 + preload

    *pw.ccr = 0;

    setBit(pw.timer->CCER, ch * 4 + pw.complementary * 2);

    if (pw.timer == TIM1
#ifdef TIM8
        || pw.timer == TIM8
#endif
#ifdef TIM20
        || pw.timer == TIM20
#endif
    ) {
        setMask(pw.timer->BDTR, TIM_BDTR_MOE);
    }

    toggleTimer<true>(pw.timer);

    pinmap_pinout(pn, PinMap_TIM);
}

inline void pwmWrite(const PwmPin& pw, const uint32_t value) {
    *pw.ccr = value;
}

#define PWM_SYNC_MAX_PINS   8
#define PWM_SYNC_MAX_TIMERS 4

struct PwmSyncState {
    struct StagedPin {
        volatile uint32_t* ccr;
        uint32_t value;
    };
    StagedPin pins[PWM_SYNC_MAX_PINS] = {};
    uint8_t pinCount = 0;

    uint8_t dirtyPins = 0ULL;

    TIM_TypeDef* timers[PWM_SYNC_MAX_TIMERS] = {};
    uint8_t timerCount = 0;
};

inline PwmSyncState pwmSync;

inline void pwmSyncRegister(PwmPin& pw) {
    if (pwmSync.pinCount < PWM_SYNC_MAX_PINS) {
        pw.syncIndex = pwmSync.pinCount;
        pwmSync.pins[pwmSync.pinCount] = {pw.ccr, 0};
        pwmSync.pinCount++;
    }

    for (uint8_t i = 0; i < pwmSync.timerCount; i++) {
        if (pwmSync.timers[i] == pw.timer) return;
    }

    if (pwmSync.timerCount < PWM_SYNC_MAX_TIMERS) {
        pwmSync.timers[pwmSync.timerCount] = pw.timer;
        pwmSync.timerCount++;
    }
}

inline void pwmSyncTimers() {
    __disable_irq();

    for (uint8_t i = 0; i < pwmSync.timerCount; i++) {
        toggleTimer<false>(pwmSync.timers[i]);
    }

    for (uint8_t i = 0; i < pwmSync.timerCount; i++) {
        pwmSync.timers[i]->CNT = 0;
        toggleTimer<true>(pwmSync.timers[i]);
    }

    __enable_irq();
}

inline void pwmStage(const PwmPin& pw, const uint32_t value) {
    pwmSync.pins[pw.syncIndex].value = value;
    setBit(pwmSync.dirtyPins, pw.syncIndex);
}

inline void pwmCommit() {
    __disable_irq();

    uint8_t dirty = pwmSync.dirtyPins;
    Bitloop(dirty) {
        const uint8_t i = GetLSB(dirty);
        *pwmSync.pins[i].ccr = pwmSync.pins[i].value;
    }
    pwmSync.dirtyPins = 0;

    __enable_irq();
}

#endif //BUCKY_PWM_H
