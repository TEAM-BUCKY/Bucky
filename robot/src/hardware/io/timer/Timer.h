#ifndef BUCKY_TIMER_H
#define BUCKY_TIMER_H

#include <stdbool.h>
#include <PinNames.h>

#include "hardware/io/mcu.h"
#include "optimizations/bitboard.h"
#include "optimizations/optimizations.h"

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

#ifdef __cplusplus
}
#endif

static FORCE_INLINE void tim_start(TIM_TypeDef* tim) {
    setMask(tim->CR1, TIM_CR1_CEN);
}

static FORCE_INLINE void tim_stop(TIM_TypeDef* tim) {
    clearMask(tim->CR1, TIM_CR1_CEN);
}

static FORCE_INLINE void tim_set_trgo(TIM_TypeDef* tim, const uint32_t mms) {
    tim->CR2 = (tim->CR2 & ~TIM_CR2_MMS_Msk) | mms << TIM_CR2_MMS_Pos;
}

static FORCE_INLINE void tim_set_dma_burst(TIM_TypeDef* tim, const uint32_t base_reg, const uint32_t count) {
    tim->DCR = (count - 1) << TIM_DCR_DBL_Pos | base_reg << TIM_DCR_DBA_Pos;
}

static FORCE_INLINE void tim_enable_update_dma(TIM_TypeDef* tim) {
    setMask(tim->DIER, TIM_DIER_UDE);
}

#endif // BUCKY_TIMER_H
