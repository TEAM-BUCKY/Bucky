#include "DMA.h"

#define GPDMA_CHANNELS  8U

typedef struct {
    bool    second;
    uint8_t index;
} ChannelInfo;

static uint32_t ll_items[2 * GPDMA_CHANNELS][3];

static ChannelInfo channel_info(const DmaChannel* ch)
{
    ChannelInfo info;
    const uintptr_t addr = (uintptr_t)ch;
    info.second = addr >= GPDMA2_Channel0_BASE;
    info.index  = (uint8_t)((addr - (info.second ? GPDMA2_Channel0_BASE : GPDMA1_Channel0_BASE)) / 0x80U);
    return info;
}

void dma_reset(DmaChannel* ch)
{
    const ChannelInfo info = channel_info(ch);
    if (info.second) __HAL_RCC_GPDMA2_CLK_ENABLE();
    else             __HAL_RCC_GPDMA1_CLK_ENABLE();

    /* GPDMA ignores EN=0 writes: suspend, wait, then reset the channel. */
    if (testMask(ch->CCR, DMA_CCR_EN)) {
        setMask(ch->CCR, DMA_CCR_SUSP);
        while (!testMask(ch->CSR, DMA_CSR_SUSPF | DMA_CSR_IDLEF)) {}
    }
    ch->CCR  = DMA_CCR_RESET;
    ch->CLLR = 0;
    ch->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF | DMA_CFCR_DTEF | DMA_CFCR_ULEF
             | DMA_CFCR_USEF | DMA_CFCR_SUSPF | DMA_CFCR_TOF;
}

void dma_setup(DmaChannel* ch, const DmaDirection dir, const DmaWidth width,
               volatile void* periph_addr, volatile void* mem_addr,
               const uint32_t count, const uint32_t request, const bool circular)
{
    const ChannelInfo info = channel_info(ch);
    dma_reset(ch);

    const bool to_periph = dir == DMA_MEM_TO_PERIPH;
    const uint32_t bytes = count << width;   /* GPDMA counts bytes, not items */
    const uint32_t src   = (uint32_t)(to_periph ? mem_addr : periph_addr);
    const uint32_t dst   = (uint32_t)(to_periph ? periph_addr : mem_addr);

    ch->CTR1 = (uint32_t)width << DMA_CTR1_SDW_LOG2_Pos
             | (uint32_t)width << DMA_CTR1_DDW_LOG2_Pos
             | (to_periph ? DMA_CTR1_SINC : DMA_CTR1_DINC);
    ch->CTR2 = request << DMA_CTR2_REQSEL_Pos
             | (to_periph ? DMA_CTR2_DREQ : 0U)
             | (circular ? DMA_CTR2_TCEM_1 : 0U);   /* TC/HT per linked-list item */
    ch->CBR1 = bytes;
    ch->CSAR = src;
    ch->CDAR = dst;

    if (circular) {
        uint32_t* item = ll_items[(info.second ? GPDMA_CHANNELS : 0U) + info.index];
        const uint32_t item_addr = (uint32_t)item;
        const uint32_t link = (item_addr & 0xFFFCU) | DMA_CLLR_UB1 | DMA_CLLR_ULL
                            | (to_periph ? DMA_CLLR_USA : DMA_CLLR_UDA);
        item[0] = bytes;
        item[1] = to_periph ? src : dst;
        item[2] = link;
        ch->CLBAR = item_addr & 0xFFFF0000U;
        ch->CLLR  = link;
    }
}

uint32_t dma_remaining(const DmaChannel* ch)
{
    const uint32_t width = (ch->CTR1 & DMA_CTR1_SDW_LOG2_Msk) >> DMA_CTR1_SDW_LOG2_Pos;
    return (ch->CBR1 & DMA_CBR1_BNDT_Msk) >> width;
}

uint32_t dma_take_events(DmaChannel* ch)
{
    const uint32_t csr = ch->CSR;
    uint32_t events = 0;
    if (csr & DMA_CSR_TCF) events |= DMA_EVT_TC;
    if (csr & DMA_CSR_HTF) events |= DMA_EVT_HT;
    ch->CFCR = csr & (DMA_CFCR_TCF | DMA_CFCR_HTF | DMA_CFCR_DTEF | DMA_CFCR_ULEF | DMA_CFCR_USEF);
    return events;
}

IRQn_Type dma_irqn(const DmaChannel* ch)
{
    const ChannelInfo info = channel_info(ch);
    return (IRQn_Type)((info.second ? GPDMA2_Channel0_IRQn : GPDMA1_Channel0_IRQn) + info.index);
}

void dma_enable(DmaChannel* ch)
{
    setMask(ch->CCR, DMA_CCR_EN);
}

void dma_enable_events(DmaChannel* ch, const uint32_t events)
{
    setMask(ch->CCR, ((events & DMA_EVT_TC) ? DMA_CCR_TCIE : 0U)
                   | ((events & DMA_EVT_HT) ? DMA_CCR_HTIE : 0U));
}
