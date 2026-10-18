#ifndef BUCKY_DMA_H
#define BUCKY_DMA_H

#include <stdbool.h>

#include <hardware/io/mcu.h>
#include <optimizations/bitboard.h>
#include <optimizations/optimizations.h>

typedef DMA_Channel_TypeDef DmaChannel;

typedef enum {
    DMA_PERIPH_TO_MEM = 0,
    DMA_MEM_TO_PERIPH = 1,
} DmaDirection;

typedef enum {
    DMA_WIDTH_8  = 0,
    DMA_WIDTH_16 = 1,
    DMA_WIDTH_32 = 2,
} DmaWidth;

#define DMA_EVT_TC  0x1U
#define DMA_EVT_HT  0x2U

#ifdef __cplusplus
extern "C" {
#endif

void dma_reset(DmaChannel* ch);

void dma_setup(DmaChannel* ch, DmaDirection dir, DmaWidth width,
               volatile void* periph_addr, volatile void* mem_addr,
               uint32_t count, uint32_t request, bool circular);

void dma_enable(DmaChannel* ch);

void dma_enable_events(DmaChannel* ch, uint32_t events);

uint32_t dma_remaining(const DmaChannel* ch);

uint32_t dma_take_events(DmaChannel* ch);

IRQn_Type dma_irqn(const DmaChannel* ch);

#ifdef __cplusplus
}
#endif

#endif // BUCKY_DMA_H
