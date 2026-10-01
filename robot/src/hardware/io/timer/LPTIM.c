#include "LPTIM.h"

#if defined(MCU_FAMILY_H5)

#include <Arduino.h>
#include <pinmap.h>

#include "optimizations/bitboard.h"

#define LPTIM_MAX_PRESC_LOG2  7U   /* /128 */

/* Enable the kernel clock and return its rate (PCLK3 for LPTIM1 by default). */
#define LPTIM_CLOCK(n)                                                \
    if (lptim == LPTIM##n) {                                          \
        __HAL_RCC_LPTIM##n##_CLK_ENABLE();                            \
        return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_LPTIM##n);    \
    }

static uint32_t lptim_clock_enable(const LPTIM_TypeDef* lptim)
{
    LPTIM_CLOCK(1)
    LPTIM_CLOCK(2)
#ifdef LPTIM3
    LPTIM_CLOCK(3)
#endif
#ifdef LPTIM4
    LPTIM_CLOCK(4)
#endif
#ifdef LPTIM5
    LPTIM_CLOCK(5)
#endif
#ifdef LPTIM6
    LPTIM_CLOCK(6)
#endif
    return 0;
}

void lptim_pin_connect(const PinName pin, const uint8_t af)
{
    pin_function(pin, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, af));
}

/* ARR/CCRx writes need ENABLE=1 and are only latched once xOK is set. */
static void write_acked(LPTIM_TypeDef* lptim, volatile uint32_t* reg, const uint32_t value,
                        const uint32_t ok_flag, const uint32_t ok_clear)
{
    lptim->ICR = ok_clear;
    *reg = value;
    while (!testMask(lptim->ISR, ok_flag)) {}
    lptim->ICR = ok_clear;
}

static void write_compare(LPTIM_TypeDef* lptim, const uint8_t channel, const uint32_t ticks)
{
    if (channel == 0U)
        write_acked(lptim, &lptim->CCR1, ticks, LPTIM_ISR_CMP1OK, LPTIM_ICR_CMP1OKCF);
    else
        write_acked(lptim, &lptim->CCR2, ticks, LPTIM_ISR_CMP2OK, LPTIM_ICR_CMP2OKCF);
}

bool lptim_pwm_start(LPTIM_TypeDef* lptim, const uint8_t channel, const uint32_t freq_hz,
                     const uint16_t duty_permille)
{
    if (freq_hz == 0U || channel > 1U) return false;

    const uint32_t clk = lptim_clock_enable(lptim);

    /* Smallest prescaler that fits the period in 16 bits keeps resolution highest. */
    uint32_t presc_log2 = 0;
    uint32_t period = clk / freq_hz;
    while (period > 0x10000U && presc_log2 < LPTIM_MAX_PRESC_LOG2) {
        presc_log2++;
        period = (clk >> presc_log2) / freq_hz;
    }
    if (period > 0x10000U || period < 2U) return false;

    const uint32_t pulse = (period * (duty_permille > 1000U ? 1000U : duty_permille) + 500U) / 1000U;

    lptim_stop(lptim);

    /* Disabled: counter configuration. Internal clock, PWM waveform, no preload
     * so the acknowledged ARR/CCR writes take effect immediately. */
    lptim->CFGR = presc_log2 << LPTIM_CFGR_PRESC_Pos;

    /* CCxSEL = 0: output, CCxP = 0: non-inverted. */
    clearMask(lptim->CCMR1, channel == 0U
        ? (LPTIM_CCMR1_CC1SEL | LPTIM_CCMR1_CC1E | LPTIM_CCMR1_CC1P_Msk)
        : (LPTIM_CCMR1_CC2SEL | LPTIM_CCMR1_CC2E | LPTIM_CCMR1_CC2P_Msk));

    /* Enabled: period, compare, then outputs and continuous counting. */
    setMask(lptim->CR, LPTIM_CR_ENABLE);
    write_acked(lptim, &lptim->ARR, period - 1U, LPTIM_ISR_ARROK, LPTIM_ICR_ARROKCF);
    write_compare(lptim, channel, pulse);

    setMask(lptim->CCMR1, channel == 0U ? LPTIM_CCMR1_CC1E : LPTIM_CCMR1_CC2E);
    setMask(lptim->CR, LPTIM_CR_CNTSTRT);
    return true;
}

void lptim_stop(LPTIM_TypeDef* lptim)
{
    clearMask(lptim->CCMR1, LPTIM_CCMR1_CC1E | LPTIM_CCMR1_CC2E);
    clearMask(lptim->CR, LPTIM_CR_ENABLE);
}

#endif // MCU_FAMILY_H5
