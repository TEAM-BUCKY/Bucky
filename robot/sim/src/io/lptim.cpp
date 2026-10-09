// Replacement for hardware/io/timer/LPTIM.c: same prescaler/period/pulse arithmetic and the same
// register writes, minus the ARROK/CMPxOK acknowledge handshakes (writes land immediately).
#include "hardware/io/timer/LPTIM.h"

#include <Arduino.h>
#include <pinmap.h>

#include "optimizations/bitboard.h"

#include "io/io.h"

#define LPTIM_MAX_PRESC_LOG2 7U

extern "C" {

void lptim_pin_connect(const PinName pin, const uint8_t af) {
    pin_function(pin, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, af));
}

bool lptim_pwm_start(LPTIM_TypeDef* lptim, const uint8_t channel, const uint32_t freq_hz,
                     const uint16_t duty_permille) {
    if (freq_hz == 0U || channel > 1U) return false;
    const auto clk = static_cast<uint32_t>(sim::io::LPTIM_CLOCK_HZ);

    uint32_t presc_log2 = 0;
    uint32_t period = clk / freq_hz;
    while (period > 0x10000U && presc_log2 < LPTIM_MAX_PRESC_LOG2) {
        presc_log2++;
        period = (clk >> presc_log2) / freq_hz;
    }
    if (period > 0x10000U || period < 2U) return false;

    const uint32_t pulse = (period * (duty_permille > 1000U ? 1000U : duty_permille) + 500U) / 1000U;

    lptim_stop(lptim);
    lptim->CFGR = presc_log2 << LPTIM_CFGR_PRESC_Pos;
    setMask(lptim->CR, LPTIM_CR_ENABLE);
    lptim->ARR = period - 1U;
    if (channel == 0U) lptim->CCR1 = pulse; else lptim->CCR2 = pulse;
    setMask(lptim->CCMR1, channel == 0U ? LPTIM_CCMR1_CC1E : LPTIM_CCMR1_CC2E);
    setMask(lptim->CR, LPTIM_CR_CNTSTRT);
    return true;
}

void lptim_stop(LPTIM_TypeDef* lptim) {
    clearMask(lptim->CCMR1, LPTIM_CCMR1_CC1E | LPTIM_CCMR1_CC2E);
    clearMask(lptim->CR, LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT);
}

}  // extern "C"
