#ifndef BUCKY_TIMER_H
#define BUCKY_TIMER_H

#include <stdbool.h>
#include <PinNames.h>

#include <hardware/io/mcu.h>
#include <optimizations/bitboard.h>
#include <optimizations/optimizations.h>

#define TIM_TRGO_OCREF(ch) (4U + (ch))

#ifdef __cplusplus
extern "C" {
#endif

void tim_clock_enable(TIM_TypeDef* tim);

uint32_t tim_clock_hz(const TIM_TypeDef* tim);

TIM_TypeDef* tim_from_pin(PinName pin, uint8_t* channel, bool* complementary);

void tim_pin_connect(PinName pin);

uint32_t tim_set_frequency(TIM_TypeDef* tim, uint32_t freq_hz);

void tim_pwm_channel_enable(TIM_TypeDef* tim, uint8_t channel, bool complementary);

TIM_TypeDef* tim_pwm_setup(PinName pin, uint32_t freq_hz, uint16_t duty_permille, uint8_t* channel);

#ifdef __cplusplus
}
#endif

static FORCE_INLINE void tim_start(TIM_TypeDef* tim) {
    setMask(tim->CR1, TIM_CR1_CEN);
}

static FORCE_INLINE void tim_stop(TIM_TypeDef* tim) {
    clearMask(tim->CR1, TIM_CR1_CEN);
}

// Latch preloaded PSC/ARR/CCR values and clear the resulting flags.
static FORCE_INLINE void tim_load(TIM_TypeDef* tim) {
    tim->EGR = TIM_EGR_UG;
    tim->SR  = 0;
}

static FORCE_INLINE volatile uint32_t* tim_ccr(TIM_TypeDef* tim, const uint8_t channel) {
    return &tim->CCR1 + channel;   // CCR1..CCR4 are contiguous
}

static FORCE_INLINE void tim_set_trgo(TIM_TypeDef* tim, const uint32_t mms) {
    tim->CR2 = (tim->CR2 & ~TIM_CR2_MMS_Msk) | mms << TIM_CR2_MMS_Pos;
}

#endif // BUCKY_TIMER_H
