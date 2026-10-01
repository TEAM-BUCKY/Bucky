#include "ADC.h"
#include <Arduino.h>
#include "optimizations/bitboard.h"

void adc_clock_enable(const ADC_TypeDef* adc)
{
#if defined(MCU_FAMILY_G4)
    if (adc == ADC1 || adc == ADC2) __HAL_RCC_ADC12_CLK_ENABLE();
    else                            __HAL_RCC_ADC345_CLK_ENABLE();
#elif defined(MCU_FAMILY_H5)
    (void)adc;
    __HAL_RCC_ADC_CLK_ENABLE();
#endif
}

ADC_Common_TypeDef* adc_common(const ADC_TypeDef* adc)
{
#if defined(MCU_FAMILY_G4)
    return (adc == ADC1 || adc == ADC2) ? ADC12_COMMON : ADC345_COMMON;
#else
    (void)adc;
    return ADC12_COMMON;
#endif
}

void adc_set_sync_clock(const ADC_TypeDef* adc, const uint32_t ckmode)
{
    // CKMODE may only change while every ADC on the common block is disabled,
    // so skip the write when a sibling ADC already set the same clock.
    ADC_Common_TypeDef* common = adc_common(adc);
    const uint32_t ccr = (common->CCR & ~ADC_CCR_CKMODE_Msk) | ckmode << ADC_CCR_CKMODE_Pos;
    if (common->CCR != ccr) common->CCR = ccr;
}

void adc_disable(ADC_TypeDef* adc) {
    if (testMask(adc->CR, ADC_CR_ADSTART)) {
        setMask(adc->CR, ADC_CR_ADSTP);
        while (testMask(adc->CR, ADC_CR_ADSTP)) {}
    }
    if (testMask(adc->CR, ADC_CR_ADEN)) {
        setMask(adc->CR, ADC_CR_ADDIS);
        while (testMask(adc->CR, ADC_CR_ADEN)) {}
    }
}

void adc_init_triggered(ADC_TypeDef* adc, const uint32_t channel, const uint32_t extsel, const uint32_t smp,
                        const AdcTriggerEdge edge) {
    clearMask(adc->CR, ADC_CR_DEEPPWD);

    // Enable internal voltage regulator
    setMask(adc->CR, ADC_CR_ADVREGEN);
    delayMicroseconds(20); // tADCVREG_STUP

    // Single-ended calibration
    clearMask(adc->CR, ADC_CR_ADCALDIF);
    setMask(adc->CR, ADC_CR_ADCAL);
    while (testMask(adc->CR, ADC_CR_ADCAL)) {}

    adc->IER = 0;
    adc->CFGR2 = 0;

    adc->CFGR = ADC_CFGR_DMAEN
              | ADC_CFGR_DMACFG
              | ADC_CFGR_OVRMOD
              | extsel << ADC_CFGR_EXTSEL_Pos
              | (uint32_t)edge << ADC_CFGR_EXTEN_Pos;

    adc->SQR1 = channel << ADC_SQR1_SQ1_Pos;

    if (channel < 10) {
        const uint32_t shift = channel * 3;
        writeField(adc->SMPR1, 7U, shift, smp);
    } else {
        const uint32_t shift = (channel - 10) * 3;
        writeField(adc->SMPR2, 7U, shift, smp);
    }

    adc->ISR = ADC_ISR_ADRDY;
    setMask(adc->CR, ADC_CR_ADEN);
    while (!testMask(adc->ISR, ADC_ISR_ADRDY)) {}

    setMask(adc->CR, ADC_CR_ADSTART);
}
