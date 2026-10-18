#ifndef BUCKY_ADC_H
#define BUCKY_ADC_H

#include <hardware/io/mcu.h>

#define ADC_EXTSEL_FROM_HAL(trig) (((trig) & ADC_CFGR_EXTSEL_Msk) >> ADC_CFGR_EXTSEL_Pos)

#ifdef __cplusplus
extern "C" {
#endif

void adc_clock_enable(const ADC_TypeDef* adc);
ADC_Common_TypeDef* adc_common(const ADC_TypeDef* adc);

void adc_set_sync_clock(const ADC_TypeDef* adc, uint32_t ckmode);

void adc_disable(ADC_TypeDef* adc);
typedef enum {
    ADC_TRIGGER_RISING  = 1,
    ADC_TRIGGER_FALLING = 2,
    ADC_TRIGGER_BOTH    = 3,
} AdcTriggerEdge;

void adc_init_triggered(ADC_TypeDef* adc, uint32_t channel, uint32_t extsel, uint32_t smp,
                        AdcTriggerEdge edge);

#ifdef __cplusplus
}
#endif

#endif // BUCKY_ADC_H
