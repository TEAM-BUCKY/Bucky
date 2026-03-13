#include "DMA.h"

void dma_init_mem_to_periph_32(DMA_Channel_TypeDef* channel,
                               DMAMUX_Channel_TypeDef* mux,
                               volatile void* periph_addr,
                               const void* mem_addr,
                               const uint32_t transfer_count,
                               const uint32_t mux_request) {
    channel->CCR   = 0;
    channel->CPAR  = reinterpret_cast<uint32_t>(periph_addr);
    channel->CMAR  = reinterpret_cast<uint32_t>(mem_addr);
    channel->CNDTR = transfer_count;
    channel->CCR   = DMA_CCR_MINC
                   | DMA_CCR_DIR      // mem -> periph
                   | DMA_CCR_CIRC
                   | DMA_CCR_MSIZE_1  // 32-bit memory
                   | DMA_CCR_PSIZE_1; // 32-bit peripheral
    mux->CCR = mux_request;
}

void dma_init_periph_to_mem_16(DMA_Channel_TypeDef* channel,
                               DMAMUX_Channel_TypeDef* mux,
                               volatile void* periph_addr,
                               volatile void* mem_addr,
                               const uint32_t transfer_count,
                               const uint32_t mux_request) {
    channel->CCR   = 0;
    channel->CPAR  = reinterpret_cast<uint32_t>(periph_addr);
    channel->CMAR  = reinterpret_cast<uint32_t>(mem_addr);
    channel->CNDTR = transfer_count;
    channel->CCR   = DMA_CCR_MINC
                   | DMA_CCR_CIRC
                   | DMA_CCR_MSIZE_0  // 16-bit memory
                   | DMA_CCR_PSIZE_0; // 16-bit peripheral
    mux->CCR = mux_request;
}

void dma_enable(DMA_Channel_TypeDef* channel) {
    channel->CCR |= DMA_CCR_EN;
}
