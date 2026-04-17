#include "tests.h"
#include "debug.h"
#include <Arduino.h>
#include <stm32g4xx.h>
#include "optimizations/bitboard.h"

// Bare-metal ADC test — runs BEFORE setupEnvironment() to avoid
// any interference from DMA, timers, OPAMP, or other peripherals.
// Prints individual sample values (not peak) for honest diagnostics.

static void printHex(const char* label, uint32_t val) {
    DBG_PRINT(label);
    for (int i = 28; i >= 0; i -= 4) {
        uint8_t nibble = (val >> i) & 0xF;
        DBG_PRINT((char)(nibble < 10 ? '0' + nibble : 'A' + nibble - 10));
    }
    DBG_PRINTLN();
}

static uint16_t adc_poll_one(ADC_TypeDef* adc) {
    setMask(adc->CR, ADC_CR_ADSTART);
    while (!testMask(adc->ISR, ADC_ISR_EOC)) {}
    return adc->DR;
}

static void adc_setup(ADC_TypeDef* adc, uint32_t channel, uint32_t smp) {
    // Disable if running
    if (testMask(adc->CR, ADC_CR_ADSTART)) {
        setMask(adc->CR, ADC_CR_ADSTP);
        while (testMask(adc->CR, ADC_CR_ADSTP)) {}
    }
    if (testMask(adc->CR, ADC_CR_ADEN)) {
        setMask(adc->CR, ADC_CR_ADDIS);
        while (testMask(adc->CR, ADC_CR_ADEN)) {}
    }

    // Exit deep power-down, enable voltage regulator
    clearMask(adc->CR, ADC_CR_DEEPPWD);
    setMask(adc->CR, ADC_CR_ADVREGEN);
    delayMicroseconds(20);

    // Calibrate single-ended
    clearMask(adc->CR, ADC_CR_ADCALDIF);
    setMask(adc->CR, ADC_CR_ADCAL);
    while (testMask(adc->CR, ADC_CR_ADCAL)) {}

    // Software trigger, no DMA, 12-bit right-aligned, overrun overwrite
    adc->IER   = 0;
    adc->CFGR  = ADC_CFGR_OVRMOD;
    adc->CFGR2 = 0;
    adc->SQR1  = channel << 6;

    if (channel < 10)
        writeField(adc->SMPR1, 7U, channel * 3, smp);
    else
        writeField(adc->SMPR2, 7U, (channel - 10) * 3, smp);

    // Enable
    adc->ISR = ADC_ISR_ADRDY;
    setMask(adc->CR, ADC_CR_ADEN);
    while (!testMask(adc->ISR, ADC_ISR_ADRDY)) {}
}

// This test is called from main() BEFORE setupEnvironment().
// It only needs Serial — no other peripherals.
void testADCRaw(const TestContext&) {
    DBG_PRINTLN("=== Isolated ADC Test (no DMA, no timers, no OPAMP) ===");
    DBG_PRINTLN("Pull PB14 or PA4 to GND/3V3 to verify.");
    DBG_PRINTLN();

    // ---- Clock enables (only what we need) ----
    setMask(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);
    setMask(RCC->AHB2ENR, RCC_AHB2ENR_ADC12EN
                         | RCC_AHB2ENR_ADC345EN
                         | RCC_AHB2ENR_DAC1EN
                         | RCC_AHB2ENR_DAC2EN
                         | RCC_AHB2ENR_GPIOAEN
                         | RCC_AHB2ENR_GPIOBEN);
    __DSB();

    // ---- DAC outputs: 3.3V / 2 = 1.65V on PA5 and PA6 ----
    // PA5 = DAC1_OUT2, PA6 = DAC2_OUT1 (both need analog GPIO mode)
    writeField(GPIOA->MODER, 3U, 5 * 2, 3U);    // PA5 = analog (DAC1_CH2)
    writeField(GPIOA->MODER, 3U, 6 * 2, 3U);    // PA6 = analog (DAC2_CH1)

    // DAC1 channel 2 (PA5): mode 000 = external pin + buffer enabled (reset default)
    DAC1->MCR = (DAC1->MCR & ~(7U << 16));        // MODE2[2:0] = 000
    DAC1->CR |= DAC_CR_EN2;                        // enable BEFORE writing data
    delayMicroseconds(10);                          // tWAKEUP
    DAC1->DHR12R2 = 2048;                          // 2048/4096 * 3.3V = 1.65V

    // DAC2 channel 1 (PA6)
    DAC2->MCR = (DAC2->MCR & ~(7U << 0));          // MODE1[2:0] = 000
    DAC2->CR |= DAC_CR_EN1;
    delayMicroseconds(10);
    DAC2->DHR12R1 = 2048;

    delayMicroseconds(100);                         // let outputs settle

    // PA7 has no DAC — drive as GPIO output HIGH (3.3V reference)
    writeField(GPIOA->MODER, 3U, 7 * 2, 1U);     // PA7 = output
    GPIOA->BSRR = (1U << 7);                       // PA7 = HIGH

    DBG_PRINTLN("DAC: PA5=1.65V  PA6=1.65V  PA7=3.3V (GPIO)");
    printHex("DAC1 CR      = 0x", DAC1->CR);
    printHex("DAC1 MCR     = 0x", DAC1->MCR);
    printHex("DAC1 DOR2    = 0x", DAC1->DOR2);
    printHex("DAC2 CR      = 0x", DAC2->CR);
    printHex("DAC2 DOR1    = 0x", DAC2->DOR1);
    printHex("GPIOA MODER  = 0x", GPIOA->MODER);

    // ---- Disconnect PB14 from ALL internal analog muxes ----
    // PB14 is OPAMP2_VINP, OPAMP5_VINP, and COMP7_INP.
    // Each creates ~18MΩ leakage to VDD through its input switch.
    // Set VP_SEL=11 (internal DAC) on both OPAMPs to disconnect PB14.
    setMask(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);  // OPAMP clock
    __DSB();
    OPAMP2->CSR = OPAMP_CSR_VPSEL_1 | OPAMP_CSR_VPSEL_0;  // VP_SEL=11 → DAC3_CH2
    OPAMP5->CSR = OPAMP_CSR_VPSEL_1 | OPAMP_CSR_VPSEL_0;  // VP_SEL=11 → DAC4_CH2
    // COMP7: ensure disabled and route INP away from PB14 if possible
    COMP7->CSR = 0;  // EN=0, INPSEL=0 — disabled

    // ---- GPIO: analog mode, no pulls ----
    writeField(GPIOB->MODER, 3U, 14 * 2, 3U);   // PB14 = analog
    writeField(GPIOB->PUPDR, 3U, 14 * 2, 0U);    // no pull
    writeField(GPIOB->OTYPER, 1U, 14, 0U);        // push-pull (irrelevant in analog but clean)
    writeField(GPIOA->MODER, 3U, 4 * 2, 3U);     // PA4  = analog
    writeField(GPIOA->PUPDR, 3U, 4 * 2, 0U);      // no pull

    // ---- Enable internal VREFBUF (provides VREF+ if pin 7 is unconnected) ----
    // On LQFP64, VREF+ is a separate pin from VDDA. If not wired to 3.3V,
    // the ADC has no reference and reads 4095 for everything.
    // VREFBUF provides 2.048V (VRS=0) or 2.5V (VRS=1) as internal VREF+.
    VREFBUF->CSR &= ~VREFBUF_CSR_HIZ;   // exit high-impedance mode
    VREFBUF->CSR |= VREFBUF_CSR_ENVR;   // enable voltage reference buffer
    while (!(VREFBUF->CSR & VREFBUF_CSR_VRR)) {}  // wait until ready
    DBG_PRINTLN("VREFBUF enabled (2.048V internal VREF+)");
    DBG_PRINTLN("NOTE: ADC range is now 0-2.048V, not 0-3.3V");
    DBG_PRINTLN("      expected GND=0, 1.65V DAC~3300, 2.048V=4095");
    DBG_PRINTLN();

    // ---- ADC clock: synchronous HCLK/4 ----
    ADC12_COMMON->CCR  = (ADC12_COMMON->CCR  & ~ADC_CCR_CKMODE_Msk) | (3U << ADC_CCR_CKMODE_Pos);
    ADC345_COMMON->CCR = (ADC345_COMMON->CCR & ~ADC_CCR_CKMODE_Msk) | (3U << ADC_CCR_CKMODE_Pos);

    // ---- Register dump ----
    printHex("GPIOB MODER  = 0x", GPIOB->MODER);
    printHex("GPIOB PUPDR  = 0x", GPIOB->PUPDR);
    printHex("OPAMP2 CSR   = 0x", OPAMP2->CSR);
    printHex("ADC12  CCR   = 0x", ADC12_COMMON->CCR);
    printHex("ADC345 CCR   = 0x", ADC345_COMMON->CCR);
    DBG_PRINTLN();

    // ---- Test 1: ADC1_IN5 = PB14 (smp=5, 247.5 cycles — long settle) ----
    DBG_PRINTLN("-- ADC1 ch5 (PB14), smp=5 --");
    adc_setup(ADC1, 5, 5);
    printHex("ADC1 CR      = 0x", ADC1->CR);
    printHex("ADC1 CFGR    = 0x", ADC1->CFGR);
    printHex("ADC1 SQR1    = 0x", ADC1->SQR1);
    printHex("ADC1 SMPR1   = 0x", ADC1->SMPR1);
    printHex("ADC1 CALFACT = 0x", ADC1->CALFACT);
    for (int i = 0; i < 10; i++) {
        DBG_PRINT("  sample "); DBG_PRINT(i); DBG_PRINT(": ");
        DBG_PRINTLN(adc_poll_one(ADC1));
    }
    DBG_PRINTLN();

    // ---- Test 2: ADC4_IN4 = PB14 (the actual production ADC) ----
    DBG_PRINTLN("-- ADC4 ch4 (PB14), smp=5 --");
    adc_setup(ADC4, 4, 5);
    printHex("ADC4 CR      = 0x", ADC4->CR);
    printHex("ADC4 CFGR    = 0x", ADC4->CFGR);
    printHex("ADC4 SQR1    = 0x", ADC4->SQR1);
    printHex("ADC4 SMPR1   = 0x", ADC4->SMPR1);
    printHex("ADC4 CALFACT = 0x", ADC4->CALFACT);
    for (int i = 0; i < 10; i++) {
        DBG_PRINT("  sample "); DBG_PRINT(i); DBG_PRINT(": ");
        DBG_PRINTLN(adc_poll_one(ADC4));
    }
    DBG_PRINTLN();

    // ---- Test 3: ADC2_IN17 = PA4 (Board 1 reference) ----
    DBG_PRINTLN("-- ADC2 ch17 (PA4), smp=5 --");
    adc_setup(ADC2, 17, 5);
    for (int i = 0; i < 10; i++) {
        DBG_PRINT("  sample "); DBG_PRINT(i); DBG_PRINT(": ");
        DBG_PRINTLN(adc_poll_one(ADC2));
    }
    DBG_PRINTLN();

    // ---- Continuous loop: print individual values for PB14 via both ADCs ----
    DBG_PRINTLN("=== Continuous (ADC1_IN5 vs ADC4_IN4 for PB14) ===");
    adc_setup(ADC1, 5, 5);

    while (true) {
        // Read 4 individual samples from ADC1 ch5 (PB14)
        DBG_PRINT("ADC1_IN5:");
        for (int i = 0; i < 4; i++) {
            DBG_PRINT('\t');
            DBG_PRINT(adc_poll_one(ADC1));
        }
        DBG_PRINTLN();

        delay(500);
    }
}
