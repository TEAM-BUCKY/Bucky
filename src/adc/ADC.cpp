#include "ADC.h"
#include <Arduino.h>
#include "../bitboard/bitboard.h"

#ifndef ADC_CR_BOOST
#define ADC_CR_BOOST (1UL << 8)
#endif

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

void adc_init_triggered(ADC_TypeDef* adc, const uint32_t channel, const uint32_t extsel) {
    clearMask(adc->CR, ADC_CR_DEEPPWD);

    // Enable internal voltage regulator
    setMask(adc->CR, ADC_CR_ADVREGEN);
    delayMicroseconds(20); // tADCVREG_STUP

    // Boost mode required for fADC > 20 MHz (we use 42.5 MHz)
    setMask(adc->CR, ADC_CR_BOOST);

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
              | 1U << ADC_CFGR_EXTEN_Pos;

    adc->SQR1 = channel << 6; // SQ1 at bits [10:6]

    if (channel < 10) {
        const uint32_t shift = channel * 3;
        writeField(adc->SMPR1, 7U, shift, 2U);
    } else {
        const uint32_t shift = (channel - 10) * 3;
        writeField(adc->SMPR2, 7U, shift, 2U);
    }

    adc->ISR = ADC_ISR_ADRDY;
    setMask(adc->CR, ADC_CR_ADEN);
    while (!testMask(adc->ISR, ADC_ISR_ADRDY)) {}

    setMask(adc->CR, ADC_CR_ADSTART);
}
