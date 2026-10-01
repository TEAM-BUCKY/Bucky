#include "Timer.h"

#include <Arduino.h>
#include <PeripheralPins.h>
#include <pinmap.h>
#include <timer.h>

void tim_clock_enable(TIM_TypeDef* tim)
{
    TIM_HandleTypeDef handle = {0};
    handle.Instance = tim;
    enableTimerClock(&handle);
}

uint32_t tim_clock_hz(const TIM_TypeDef* tim)
{
    RCC_ClkInitTypeDef clk;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clk, &latency);

    const bool apb2 = getTimerClkSrc((TIM_TypeDef*)tim) == 2U;
    const uint32_t pclk = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    const uint32_t divider = apb2 ? clk.APB2CLKDivider : clk.APB1CLKDivider;
    return divider == RCC_HCLK_DIV1 ? pclk : pclk * 2U;
}

TIM_TypeDef* tim_from_pin(const PinName pin, uint8_t* channel, bool* complementary)
{
    for (const PinMap* map = PinMap_TIM; map->pin != NC; map++) {
        if (map->pin != pin) continue;
        if (channel)       *channel = (uint8_t)(STM_PIN_CHANNEL(map->function) - 1U);
        if (complementary) *complementary = STM_PIN_INVERTED(map->function) != 0U;
        return (TIM_TypeDef*)map->peripheral;
    }
    return NULL;
}

void tim_pin_connect(const PinName pin)
{
    pinmap_pinout(pin, PinMap_TIM);
}

uint32_t tim_set_frequency(TIM_TypeDef* tim, const uint32_t freq_hz)
{
    if (freq_hz == 0U) return 0U;

    const uint32_t clk   = tim_clock_hz(tim);
    const uint32_t ticks = clk / freq_hz;
    if (ticks < 2U) return 0U;

    const uint32_t psc = (ticks - 1U) / 0x10000U;
    if (psc > 0xFFFFU) return 0U;

    const uint32_t period = clk / ((psc + 1U) * freq_hz);
    tim->PSC = psc;
    tim->ARR = period - 1U;
    return period;
}

void tim_pwm_channel_enable(TIM_TypeDef* tim, const uint8_t channel, const bool complementary)
{
    volatile uint32_t* ccmr = &tim->CCMR1 + (channel >> 1);
    writeField(*ccmr, 0xFFU, (channel & 1U) * 8U, 0x68U);          /* PWM mode 1, OCxPE */

    setBit(tim->CCER, channel * 4U + (complementary ? 2U : 0U));   /* CCxE / CCxNE */
    if (complementary) setBit(tim->CCER, channel * 4U + 3U);       /* CCxNP */

    /* TIM1/8/15/16/17/20: outputs stay off until the main output enable is set. */
    if (IS_TIM_BREAK_INSTANCE(tim)) setMask(tim->BDTR, TIM_BDTR_MOE);
}

TIM_TypeDef* tim_pwm_setup(const PinName pin, const uint32_t freq_hz, const uint16_t duty_permille,
                           uint8_t* channel)
{
    uint8_t ch = 0;
    bool complementary = false;
    TIM_TypeDef* tim = tim_from_pin(pin, &ch, &complementary);
    if (tim == NULL) return NULL;

    tim_clock_enable(tim);
    tim_stop(tim);

    const uint32_t period = tim_set_frequency(tim, freq_hz);
    if (period == 0U) return NULL;

    const uint32_t duty = duty_permille > 1000U ? 1000U : duty_permille;
    *tim_ccr(tim, ch) = period * duty / 1000U;
    tim_pwm_channel_enable(tim, ch, complementary);
    setMask(tim->CR1, TIM_CR1_ARPE);
    tim_load(tim);

    tim_pin_connect(pin);
    if (channel) *channel = ch;
    return tim;
}
