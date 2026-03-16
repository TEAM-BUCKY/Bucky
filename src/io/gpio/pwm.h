#ifndef BUCKY_PWM_H
#define BUCKY_PWM_H

#include <Arduino.h>
#include <stm32g4xx.h>
#include <PeripheralPins.h>

#include "optimizations/bitboard.h"
#include "optimizations/optimizations.h"

#define PWM_SYNC_MAX_PINS   8
#define PWM_SYNC_MAX_TIMERS 4

typedef struct {
    TIM_TypeDef *timer;
    volatile uint32_t *ccr;
    uint8_t channel;
    uint8_t syncIndex;
    uint8_t complementary;
} PwmPin;

typedef struct {
    volatile uint32_t *ccr;
    uint32_t value;
} PwmStagedPin;

typedef struct {
    PwmStagedPin pins[PWM_SYNC_MAX_PINS];
    uint8_t pinCount;
    uint8_t dirtyPins;
    TIM_TypeDef *timers[PWM_SYNC_MAX_TIMERS];
    uint8_t timerCount;
} PwmSyncState;

extern PwmSyncState pwm_sync;

static FORCE_INLINE void timer_enable(TIM_TypeDef *tim)
{
    setMask(tim->CR1, TIM_CR1_CEN);
}

static FORCE_INLINE void timer_disable(TIM_TypeDef *tim)
{
    clearMask(tim->CR1, TIM_CR1_CEN);
}

static inline PwmPin pwm_pin_init(const int pin)
{
    const PinName pn = digitalPinToPinName(pin);
    const uint32_t func = pinmap_function(pn, PinMap_TIM);
    PwmPin pw;
    pw.timer = (TIM_TypeDef *)pinmap_peripheral(pn, PinMap_TIM);
    pw.channel = STM_PIN_CHANNEL(func) - 1;
    pw.complementary = STM_PIN_INVERTED(func);
    pw.ccr = &pw.timer->CCR1 + pw.channel;
    pw.syncIndex = 0xFF;
    return pw;
}

static FORCE_INLINE void pwm_enable_clock(const TIM_TypeDef *tim)
{
    switch ((uintptr_t)tim) {
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

static FORCE_INLINE void pwm_init(const PwmPin *pw, const int pin, const uint32_t freq, const uint32_t resolution)
{
    const PinName pn = digitalPinToPinName(pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    pwm_enable_clock(pw->timer);

    const uint32_t timerClk = HAL_RCC_GetPCLK1Freq();
    pw->timer->PSC = timerClk / (freq * (resolution + 1)) - 1;
    pw->timer->ARR = resolution;

    const uint8_t ch = pw->channel;

    volatile uint32_t *ccmr = &pw->timer->CCMR1 + (ch >> 1);
    const uint8_t shift = (ch & 1) * 8;
    writeField(*ccmr, 0xFFU, shift, 0x68U);

    *pw->ccr = 0;

    setBit(pw->timer->CCER, ch * 4 + pw->complementary * 2);

    if (pw->timer == TIM1
#ifdef TIM8
        || pw->timer == TIM8
#endif
#ifdef TIM20
        || pw->timer == TIM20
#endif
    ) {
        setMask(pw->timer->BDTR, TIM_BDTR_MOE);
    }

    timer_enable(pw->timer);

    pinmap_pinout(pn, PinMap_TIM);
}

static FORCE_INLINE void pwm_write(const PwmPin *pw, const uint32_t value)
{
    *pw->ccr = value;
}

static FORCE_INLINE void pwm_sync_register(PwmPin *pw)
{
    if (pwm_sync.pinCount < PWM_SYNC_MAX_PINS) {
        pw->syncIndex = pwm_sync.pinCount;
        pwm_sync.pins[pwm_sync.pinCount].ccr = pw->ccr;
        pwm_sync.pins[pwm_sync.pinCount].value = 0;
        pwm_sync.pinCount++;
    }

    for (uint8_t i = 0; i < pwm_sync.timerCount; i++) {
        if (pwm_sync.timers[i] == pw->timer) return;
    }

    if (pwm_sync.timerCount < PWM_SYNC_MAX_TIMERS) {
        pwm_sync.timers[pwm_sync.timerCount] = pw->timer;
        pwm_sync.timerCount++;
    }
}

static FORCE_INLINE void pwm_sync_timers()
{
    __disable_irq();

    for (uint8_t i = 0; i < pwm_sync.timerCount; i++) {
        timer_disable(pwm_sync.timers[i]);
    }

    for (uint8_t i = 0; i < pwm_sync.timerCount; i++) {
        pwm_sync.timers[i]->CNT = 0;
        timer_enable(pwm_sync.timers[i]);
    }

    __enable_irq();
}

static FORCE_INLINE void pwm_stage(const PwmPin *pw, const uint32_t value)
{
    pwm_sync.pins[pw->syncIndex].value = value;
    setBit(pwm_sync.dirtyPins, pw->syncIndex);
}

static FORCE_INLINE void pwm_commit()
{
    __disable_irq();

    uint8_t dirty = pwm_sync.dirtyPins;
    Bitloop(dirty) {
        uint8_t i = GetLSB(dirty);
        *pwm_sync.pins[i].ccr = pwm_sync.pins[i].value;
    }
    pwm_sync.dirtyPins = 0;

    __enable_irq();
}

#endif // BUCKY_PWM_H
