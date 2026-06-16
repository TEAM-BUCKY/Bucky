#include "tests.h"
#include "debug.h"
#include <Arduino.h>
#include <stm32g4xx.h>
#include "../sensors/IRSensor.h"

static void printHex(const char* label, uint32_t val) {
    DBG_PRINT(label);
    // Print 8-digit hex
    for (int i = 28; i >= 0; i -= 4) {
        uint8_t nibble = (val >> i) & 0xF;
        DBG_PRINT((char)(nibble < 10 ? '0' + nibble : 'A' + nibble - 10));
    }
    DBG_PRINTLN();
}

void testIR(const TestContext& ctx) {
    DBG_PRINTLN("=== IR Sensor Test (DMA) ===");
    DBG_PRINTLN();

    // One-time register dump to verify configuration
    DBG_PRINTLN("-- Board 2 register dump (ADC4 / PB14 / TIM3) --");
    printHex("GPIOB MODER     = 0x", GPIOB->MODER);
    printHex("ADC4  CR        = 0x", ADC4->CR);
    printHex("ADC4  CFGR      = 0x", ADC4->CFGR);
    printHex("ADC4  SQR1      = 0x", ADC4->SQR1);
    printHex("ADC4  SMPR1     = 0x", ADC4->SMPR1);
    printHex("ADC345 CCR      = 0x", ADC345_COMMON->CCR);
    printHex("TIM2  CR1       = 0x", TIM2->CR1);
    printHex("TIM2  ARR       = 0x", TIM2->ARR);
    printHex("TIM2  CCR1      = 0x", TIM2->CCR1);
    printHex("TIM3  CR1       = 0x", TIM3->CR1);
    printHex("TIM3  CR2       = 0x", TIM3->CR2);
    printHex("TIM3  SMCR      = 0x", TIM3->SMCR);
    printHex("TIM3  ARR       = 0x", TIM3->ARR);
    printHex("TIM3  CCR2      = 0x", TIM3->CCR2);
    printHex("DMA1_CH4 CCR    = 0x", DMA1_Channel4->CCR);
    printHex("DMA1_CH4 CNDTR  = 0x", DMA1_Channel4->CNDTR);
    printHex("DMA1_CH4 CPAR   = 0x", DMA1_Channel4->CPAR);
    printHex("DMA1_CH4 CMAR   = 0x", DMA1_Channel4->CMAR);
    printHex("DMAMUX1_CH3     = 0x", DMAMUX1_Channel3->CCR);
    printHex("OPAMP2 CSR      = 0x", OPAMP2->CSR);
    DBG_PRINTLN();

    DBG_PRINTLN("-- Board 1 register dump (ADC2 / PA4 / TIM4) --");
    printHex("GPIOA MODER     = 0x", GPIOA->MODER);
    printHex("ADC2  CR        = 0x", ADC2->CR);
    printHex("ADC2  CFGR      = 0x", ADC2->CFGR);
    printHex("ADC2  SQR1      = 0x", ADC2->SQR1);
    printHex("ADC12 CCR       = 0x", ADC12_COMMON->CCR);
    DBG_PRINTLN();

    const uint32_t count2 = ir_get_sensor_count(2);

    while (true) {
        if (!ir_has_new_frame(2, ir_get_frame_sequence(2) - 1)) {
            delay(1);
            continue;
        }

        DBG_PRINT("B2 seq="); DBG_PRINT(ir_get_frame_sequence(2));
        DBG_PRINT(" CNDTR="); DBG_PRINTLN(DMA1_Channel4->CNDTR);

        const uint16_t* b2 = ir_get_buffer(2);

        // Show max across sweeps per channel (same logic as IRBallProcessor)
        DBG_PRINT("B2 MAX: ");
        for (uint32_t ch = 0; ch < count2; ch++) {
            uint16_t maxVal = 0;
            for (uint32_t sweep = 0; sweep < IR_SWEEPS_PER_CYCLE; sweep++) {
                uint16_t v = b2[sweep * IR_MUX_CHANNELS + ch];
                if (v > maxVal) maxVal = v;
            }
            if (ch > 0) DBG_PRINT('\t');
            DBG_PRINT(maxVal);
        }
        DBG_PRINTLN();

        // Also show each sweep raw for channel 0 to see hit rate
        DBG_PRINT("B2 CH0: ");
        for (uint32_t sweep = 0; sweep < IR_SWEEPS_PER_CYCLE; sweep++) {
            if (sweep > 0) DBG_PRINT('\t');
            DBG_PRINT(b2[sweep * IR_MUX_CHANNELS]);
        }
        DBG_PRINTLN();
        DBG_PRINTLN();

        delay(500);
    }
}
