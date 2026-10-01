#include "LPTIM.h"

#if defined(MCU_FAMILY_H5)

#include <Arduino.h>
#include <pinmap.h>

#include "optimizations/bitboard.h"

#define LPTIM_MAX_PRESC_LOG2  7U   /* /128 */

void lptim_clock_enable(const LPTIM_TypeDef* lptim)
{
    if      (lptim == LPTIM1) __HAL_RCC_LPTIM1_CLK_ENABLE();
    else if (lptim == LPTIM2) __HAL_RCC_LPTIM2_CLK_ENABLE();
#ifdef LPTIM3
    else if (lptim == LPTIM3) __HAL_RCC_LPTIM3_CLK_ENABLE();
#endif
#ifdef LPTIM4
    else if (lptim == LPTIM4) __HAL_RCC_LPTIM4_CLK_ENABLE();
#endif
#ifdef LPTIM5
    else if (lptim == LPTIM5) __HAL_RCC_LPTIM5_CLK_ENABLE();
#endif
#ifdef LPTIM6
    else if (lptim == LPTIM6) __HAL_RCC_LPTIM6_CLK_ENABLE();
#endif
}

uint32_t lptim_clock_hz(const LPTIM_TypeDef* lptim)
{
    if      (lptim == LPTIM1) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_LPTIM1);
    else if (lptim == LPTIM2) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_LPTIM2);
#ifdef LPTIM3
    else if (lptim == LPTIM3) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_LPTIM3);
#endif
#ifdef LPTIM4
    else if (lptim == LPTIM4) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_LPTIM4);
#endif
#ifdef LPTIM5
    else if (lptim == LPTIM5) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_LPTIM5);
#endif
#ifdef LPTIM6
    else if (lptim == LPTIM6) return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_LPTIM6);
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
                     const uint16_t duty_permille, const bool invert, LptimPwmInfo* info)
{
    if (freq_hz == 0U || channel > 1U) return false;

    lptim_clock_enable(lptim);
    const uint32_t clk = lptim_clock_hz(lptim);

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

    const uint32_t ccmr_mask = channel == 0U
        ? (LPTIM_CCMR1_CC1SEL | LPTIM_CCMR1_CC1E | LPTIM_CCMR1_CC1P_Msk)
        : (LPTIM_CCMR1_CC2SEL | LPTIM_CCMR1_CC2E | LPTIM_CCMR1_CC2P_Msk);
    const uint32_t polarity = invert
        ? (channel == 0U ? (1UL << LPTIM_CCMR1_CC1P_Pos) : (1UL << LPTIM_CCMR1_CC2P_Pos))
        : 0U;
    lptim->CCMR1 = (lptim->CCMR1 & ~ccmr_mask) | polarity;   /* CCxSEL = 0: output */

    /* Enabled: period, compare, then outputs and continuous counting. */
    setMask(lptim->CR, LPTIM_CR_ENABLE);
    write_acked(lptim, &lptim->ARR, period - 1U, LPTIM_ISR_ARROK, LPTIM_ICR_ARROKCF);
    write_compare(lptim, channel, pulse);

    setMask(lptim->CCMR1, channel == 0U ? LPTIM_CCMR1_CC1E : LPTIM_CCMR1_CC2E);
    setMask(lptim->CR, LPTIM_CR_CNTSTRT);

    if (info) {
        info->tick_hz = clk >> presc_log2;
        info->period  = period;
    }
    return true;
}

void lptim_pwm_set_pulse(LPTIM_TypeDef* lptim, const uint8_t channel, const uint32_t ticks)
{
    write_compare(lptim, channel, ticks);
}

void lptim_stop(LPTIM_TypeDef* lptim)
{
    clearMask(lptim->CCMR1, LPTIM_CCMR1_CC1E | LPTIM_CCMR1_CC2E);
    clearMask(lptim->CR, LPTIM_CR_ENABLE);
}

#endif // MCU_FAMILY_H5
