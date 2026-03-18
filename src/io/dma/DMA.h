#ifndef BUCKY_DMA_H
#define BUCKY_DMA_H

#include <stm32g4xx.h>

#include "optimizations/bitboard.h"
#include "optimizations/optimizations.h"

/* DMAMUX request IDs (RM0440 Table 91) */
#define DMAMUX_REQ_ADC_1    5
#define DMAMUX_REQ_ADC_2    36
#define DMAMUX_REQ_TIM3_UP  65
#define DMAMUX_REQ_TIM4_UP  71

#ifdef __cplusplus
extern "C" {
#endif

void dma_init_mem_to_periph_32(DMA_Channel_TypeDef* channel,
                               DMAMUX_Channel_TypeDef* mux,
                               volatile void* periph_addr,
                               const void* mem_addr,
                               uint32_t transfer_count,
                               uint32_t mux_request);

void dma_init_periph_to_mem_16(DMA_Channel_TypeDef* channel,
                               DMAMUX_Channel_TypeDef* mux,
                               volatile void* periph_addr,
                               volatile void* mem_addr,
                               uint32_t transfer_count,
                               uint32_t mux_request);

#ifdef __cplusplus
}
#endif

static FORCE_INLINE void dma_enable(DMA_Channel_TypeDef* channel)
{
    setMask(channel->CCR, DMA_CCR_EN);
}

#endif // BUCKY_DMA_H
