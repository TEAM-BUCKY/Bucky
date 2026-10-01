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
