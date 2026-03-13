#ifndef BUCKY_ADC_H
#define BUCKY_ADC_H

#include <stm32g4xx.h>

// ADC EXTSEL values for ADC12 regular triggers (RM0440 Table 73)
namespace ADCExtSel {
    constexpr uint32_t TIM3_TRGO = 4;   // 00100
    constexpr uint32_t TIM4_TRGO = 12;  // 01100
}

// Stop any ongoing conversion and disable the ADC.
void adc_disable(ADC_TypeDef* adc);

// Configure an ADC for single-channel external-triggered DMA circular mode.
void adc_init_triggered(ADC_TypeDef* adc, uint32_t channel, uint32_t extsel);

#endif // BUCKY_ADC_H
