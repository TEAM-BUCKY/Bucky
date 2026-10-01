#ifndef BUCKY_TIMER_H
#define BUCKY_TIMER_H

#include <stdbool.h>
#include <PinNames.h>

#include "hardware/io/mcu.h"
#include "optimizations/bitboard.h"
#include "optimizations/optimizations.h"

// TRGO source (CR2.MMS) for "OCxREF" of 0-based channel ch.
#define TIM_TRGO_OCREF(ch) (4U + (ch))

#ifdef __cplusplus
extern "C" {
#endif

// Enable the timer's RCC clock (any family, any instance).
void tim_clock_enable(TIM_TypeDef* tim);

// Input clock of the timer's counter in Hz (APB clock, doubled when the APB
// prescaler is not 1).
uint32_t tim_clock_hz(const TIM_TypeDef* tim);

// Look up the timer and 0-based channel a pin routes to. `pin` may carry an
// _ALTn suffix to pick between timers sharing the pad (e.g. PB_14_ALT2 = TIM12).
// Returns NULL when the pin has no timer function.
TIM_TypeDef* tim_from_pin(PinName pin, uint8_t* channel, bool* complementary);

// Route a pin to its timer alternate function.
void tim_pin_connect(PinName pin);

// Set PSC/ARR so the counter wraps at freq_hz, using the smallest prescaler
// (finest duty resolution) that fits ARR in 16 bits. Returns the period in
// ticks (ARR + 1), or 0 when freq_hz cannot be produced.
uint32_t tim_set_frequency(TIM_TypeDef* tim, uint32_t freq_hz);

// PWM mode 1 with CCR preload on a channel, and its output enabled. A
// complementary (CHxN) output is inverted so it follows OCxREF like CHx does.
void tim_pwm_channel_enable(TIM_TypeDef* tim, uint8_t channel, bool complementary);

// Configure PWM at freq_hz / duty_permille (0..1000) on the timer channel
// behind `pin` and connect the pin. The counter is left stopped so the caller
// can finish setup (e.g. TRGO) before tim_start(). Returns NULL when the pin
// has no timer function or the frequency is out of range.
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
