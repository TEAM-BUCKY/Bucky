#ifndef BUCKY_ADC_H
#define BUCKY_ADC_H

#include <stm32g4xx.h>

// External trigger selections (shared across all ADCs on G4)
#define ADC_EXTSEL_TIM3_TRGO      4
#define ADC_EXTSEL_TIM4_TRGO      12

#define DMAMUX_REQ_ADC_4          38

#ifdef __cplusplus
extern "C" {
#endif

void adc_disable(ADC_TypeDef *adc);
void adc_init_triggered(ADC_TypeDef *adc, uint32_t channel, uint32_t extsel, uint32_t smp);

#ifdef __cplusplus
}
#endif

#endif // BUCKY_ADC_H
