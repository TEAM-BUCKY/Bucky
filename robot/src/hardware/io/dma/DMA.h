#ifndef BUCKY_DMA_H
#define BUCKY_DMA_H

#include <stdbool.h>

#include "hardware/io/mcu.h"
#include "optimizations/bitboard.h"
#include "optimizations/optimizations.h"

// Family-neutral DMA channel control.
//
// G4:  DMA1/DMA2 channels, request routed through the matching DMAMUX1 channel.
// H5:  GPDMA1 channels, request selected in CTR2. Circular mode is built from a
//      single linked-list item that reloads the channel onto itself.
//
// Request IDs come straight from the HAL headers (DMA_REQUEST_* on G4,
// GPDMA1_REQUEST_* on H5) so nothing here hardcodes a reference-manual table.

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

/* Portable interrupt / flag bits. */
#define DMA_EVT_TC  0x1U
#define DMA_EVT_HT  0x2U

#ifdef __cplusplus
extern "C" {
#endif

// Stop the channel and leave it disabled with flags cleared.
void dma_reset(DmaChannel* ch);

// Configure (but do not enable) a transfer of `count` data items.
void dma_setup(DmaChannel* ch, DmaDirection dir, DmaWidth width,
               volatile void* periph_addr, volatile void* mem_addr,
               uint32_t count, uint32_t request, bool circular);

void dma_enable(DmaChannel* ch);

// Enable TC and/or HT interrupts (DMA_EVT_*) on a configured channel.
void dma_enable_events(DmaChannel* ch, uint32_t events);

// Data items still to transfer in the current block.
uint32_t dma_remaining(const DmaChannel* ch);

// Read and clear the channel's pending DMA_EVT_* flags. Call from its ISR.
uint32_t dma_take_events(DmaChannel* ch);

IRQn_Type dma_irqn(const DmaChannel* ch);

// Legacy helpers kept for the IR sensor code.
void dma_init_mem_to_periph_32(DmaChannel* ch, volatile void* periph_addr, const void* mem_addr,
                               uint32_t transfer_count, uint32_t request);

void dma_init_periph_to_mem_16(DmaChannel* ch, volatile void* periph_addr, volatile void* mem_addr,
                               uint32_t transfer_count, uint32_t request);

#ifdef __cplusplus
}
#endif

#endif // BUCKY_DMA_H
