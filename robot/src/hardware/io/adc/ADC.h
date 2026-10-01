#ifndef BUCKY_ADC_H
#define BUCKY_ADC_H

#include "hardware/io/mcu.h"

// External trigger index (CFGR.EXTSEL) from a HAL ADC_EXTERNALTRIG_* constant,
// so trigger sources come from the family's HAL instead of a hand-typed table.
#define ADC_EXTSEL_FROM_HAL(trig) (((trig) & ADC_CFGR_EXTSEL_Msk) >> ADC_CFGR_EXTSEL_Pos)

#if defined(MCU_FAMILY_G4)
// G4 ADC345 trigger numbering, verified on the G474 board (not in the HAL's
// ADC12-oriented constants).
#define ADC_EXTSEL_TIM3_TRGO      4
#define ADC_EXTSEL_TIM4_TRGO      12
#endif

#ifdef __cplusplus
extern "C" {
#endif

void adc_clock_enable(const ADC_TypeDef* adc);
ADC_Common_TypeDef* adc_common(const ADC_TypeDef* adc);

// Synchronous ADC clock (CKMODE): 1 = HCLK/1, 2 = HCLK/2, 3 = HCLK/4.
void adc_set_sync_clock(const ADC_TypeDef* adc, uint32_t ckmode);

void adc_disable(ADC_TypeDef* adc);
typedef enum {
    ADC_TRIGGER_RISING  = 1,
    ADC_TRIGGER_FALLING = 2,
    ADC_TRIGGER_BOTH    = 3,
} AdcTriggerEdge;

// Continuous DMA stream of one channel, one conversion per trigger edge.
void adc_init_triggered(ADC_TypeDef* adc, uint32_t channel, uint32_t extsel, uint32_t smp,
                        AdcTriggerEdge edge);

#ifdef __cplusplus
}
#endif

#endif // BUCKY_ADC_H
