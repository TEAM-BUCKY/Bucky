#ifndef BUCKY_ADC_H
#define BUCKY_ADC_H

#include <stm32g4xx.h>

namespace ADCExtSel {
    constexpr uint32_t TIM3_TRGO = 4;   // 00100
    constexpr uint32_t TIM4_TRGO = 12;  // 01100
}

void adc_disable(ADC_TypeDef* adc);

void adc_init_triggered(ADC_TypeDef* adc, uint32_t channel, uint32_t extsel);

#endif // BUCKY_ADC_H
