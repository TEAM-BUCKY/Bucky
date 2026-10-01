#include "DMA.h"

#if defined(MCU_FAMILY_G4)

/* ---------------------------------------------------------------- G4 ----- */

typedef struct {
    DMA_TypeDef* dma;
    uint8_t      index;   /* 0-based channel index within its controller */
    uint8_t      mux;     /* DMAMUX1 output channel */
} ChannelInfo;

static ChannelInfo channel_info(const DmaChannel* ch)
{
    ChannelInfo info;
    const uintptr_t addr = (uintptr_t)ch;
    const bool second = addr >= DMA2_Channel1_BASE;
    info.dma   = second ? DMA2 : DMA1;
    info.index = (uint8_t)((addr - (second ? DMA2_Channel1_BASE : DMA1_Channel1_BASE)) / 0x14U);
    info.mux   = (uint8_t)(info.index + (second ? 8U : 0U));
    return info;
}

static void dma_clock_enable(const ChannelInfo* info)
{
    setMask(RCC->AHB1ENR, (info->dma == DMA1 ? RCC_AHB1ENR_DMA1EN : RCC_AHB1ENR_DMA2EN)
                          | RCC_AHB1ENR_DMAMUX1EN);
    __DSB();
}

void dma_reset(DmaChannel* ch)
{
    const ChannelInfo info = channel_info(ch);
    dma_clock_enable(&info);
    ch->CCR = 0;
    info.dma->IFCR = 0xFUL << (info.index * 4U);
}

void dma_setup(DmaChannel* ch, const DmaDirection dir, const DmaWidth width,
               volatile void* periph_addr, volatile void* mem_addr,
               const uint32_t count, const uint32_t request, const bool circular)
{
    const ChannelInfo info = channel_info(ch);
    dma_reset(ch);

    ch->CPAR  = (uint32_t)periph_addr;
    ch->CMAR  = (uint32_t)mem_addr;
    ch->CNDTR = count;
    ch->CCR   = DMA_CCR_MINC
              | (dir == DMA_MEM_TO_PERIPH ? DMA_CCR_DIR : 0U)
              | (circular ? DMA_CCR_CIRC : 0U)
              | (uint32_t)width << DMA_CCR_MSIZE_Pos
              | (uint32_t)width << DMA_CCR_PSIZE_Pos;

    ((DMAMUX_Channel_TypeDef*)(DMAMUX1_Channel0_BASE + info.mux * 4U))->CCR = request;
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

uint32_t dma_remaining(const DmaChannel* ch)
{
    return ch->CNDTR;
}

uint32_t dma_take_events(DmaChannel* ch)
{
    const ChannelInfo info = channel_info(ch);
    const uint32_t shift = info.index * 4U;
    const uint32_t isr = info.dma->ISR >> shift;
    uint32_t events = 0;
    if (isr & DMA_ISR_TCIF1) events |= DMA_EVT_TC;
    if (isr & DMA_ISR_HTIF1) events |= DMA_EVT_HT;
    info.dma->IFCR = (isr & (DMA_ISR_TCIF1 | DMA_ISR_HTIF1 | DMA_ISR_TEIF1)) << shift;
    return events;
}

IRQn_Type dma_irqn(const DmaChannel* ch)
{
    const ChannelInfo info = channel_info(ch);
    if (info.dma == DMA1)
        return info.index < 7U ? (IRQn_Type)(DMA1_Channel1_IRQn + info.index) : DMA1_Channel8_IRQn;
    return info.index < 5U ? (IRQn_Type)(DMA2_Channel1_IRQn + info.index)
                           : (IRQn_Type)(DMA2_Channel6_IRQn + (info.index - 5U));
}

#elif defined(MCU_FAMILY_H5)

/* ---------------------------------------------------------------- H5 ----- */

#define GPDMA_CHANNELS  8U

typedef struct {
    bool    second;
    uint8_t index;
} ChannelInfo;

/* One self-referencing linked-list item per channel for circular transfers:
 * reloads CBR1, the incrementing address (CSAR or CDAR) and CLLR itself. */
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

void dma_enable(DmaChannel* ch)
{
    setMask(ch->CCR, DMA_CCR_EN);
}

void dma_enable_events(DmaChannel* ch, const uint32_t events)
{
    setMask(ch->CCR, ((events & DMA_EVT_TC) ? DMA_CCR_TCIE : 0U)
                   | ((events & DMA_EVT_HT) ? DMA_CCR_HTIE : 0U));
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

#endif

void dma_init_mem_to_periph_32(DmaChannel* ch, volatile void* periph_addr, const void* mem_addr,
                               const uint32_t transfer_count, const uint32_t request)
{
    dma_setup(ch, DMA_MEM_TO_PERIPH, DMA_WIDTH_32, periph_addr, (volatile void*)mem_addr,
              transfer_count, request, true);
}

void dma_init_periph_to_mem_16(DmaChannel* ch, volatile void* periph_addr, volatile void* mem_addr,
                               const uint32_t transfer_count, const uint32_t request)
{
    dma_setup(ch, DMA_PERIPH_TO_MEM, DMA_WIDTH_16, periph_addr, mem_addr,
              transfer_count, request, true);
}
