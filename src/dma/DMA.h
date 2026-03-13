#ifndef BUCKY_DMA_H
#define BUCKY_DMA_H

#include <stm32g4xx.h>

#include "bitboard/bitboard.h"

// DMAMUX request IDs (RM0440 Table 91)
namespace DMAMux {
    constexpr uint32_t ADC_1    = 5;
    constexpr uint32_t ADC_2    = 36;
    constexpr uint32_t TIM3_UP  = 65;
    constexpr uint32_t TIM4_UP  = 71;
}

// Configure a DMA channel for circular memory-to-peripheral 32-bit transfers.
void dma_init_mem_to_periph_32(DMA_Channel_TypeDef* channel,
                               DMAMUX_Channel_TypeDef* mux,
                               volatile void* periph_addr,
                               const void* mem_addr,
                               uint32_t transfer_count,
                               uint32_t mux_request);

// Configure a DMA channel for circular peripheral-to-memory 16-bit transfers.
void dma_init_periph_to_mem_16(DMA_Channel_TypeDef* channel,
                               DMAMUX_Channel_TypeDef* mux,
                               volatile void* periph_addr,
                               volatile void* mem_addr,
                               uint32_t transfer_count,
                               uint32_t mux_request);

inline void dma_enable(DMA_Channel_TypeDef *channel) {
    setMask(channel->CCR, DMA_CCR_EN);
}

#endif // BUCKY_DMA_H
