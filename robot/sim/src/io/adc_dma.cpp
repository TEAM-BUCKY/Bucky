// Replacements for hardware/io/adc/ADC.c and hardware/io/dma/DMA.c.
//
// The ADC keeps its configuration in its own registers (CFGR: EXTSEL/EXTEN/DMAEN, SQR1: channel,
// CR: ADEN/ADSTART) exactly where the real driver writes it, and the simulator converts only when
// that configuration matches the trigger that fired. DMA channels move each conversion from the
// ADC's DR into the configured memory buffer (circular or one-shot) and report what is left.
#include "hardware/io/adc/ADC.h"
#include "hardware/io/dma/DMA.h"

#include <Arduino.h>
#include <PeripheralPins.h>
#include <pinmap.h>

#include "core/world.h"
#include "io/io.h"

#define ADC_CFGR_DMAEN   (0x1UL << 0)
#define ADC_CFGR_DMACFG  (0x1UL << 1)
#define ADC_CFGR_OVRMOD  (0x1UL << 12)
#define ADC_CR_ADSTART   (0x1UL << 2)
#define ADC_SQR1_SQ1_Pos 6U

namespace {

struct DmaState {
    bool configured = false;
    bool enabled = false;
    DmaDirection dir = DMA_PERIPH_TO_MEM;
    DmaWidth width = DMA_WIDTH_8;
    volatile void* periph = nullptr;
    volatile void* mem = nullptr;
    uint32_t count = 0;
    uint32_t pos = 0;
    uint32_t request = 0;
    bool circular = false;
};

DmaState dma[8];

int dma_index(const DmaChannel* ch) {
    for (int i = 0; i < 8; i++)
        if (ch == &sim_GPDMA1_Channel[i]) return i;
    return -1;
}

void reset_dma() {
    for (auto& d : dma) d = {};
}

const bool registered = [] {
    sim::world().reset_hooks.emplace_back(reset_dma);
    return true;
}();

uint32_t adc_request(const ADC_TypeDef* adc) {
    return adc == ADC1 ? GPDMA1_REQUEST_ADC1 : GPDMA1_REQUEST_ADC2;
}

}  // namespace

extern "C" {

// ---- ADC ---------------------------------------------------------------------------------------

void adc_clock_enable(const ADC_TypeDef* /*adc*/) {}

ADC_Common_TypeDef* adc_common(const ADC_TypeDef* /*adc*/) { return ADC12_COMMON; }

void adc_set_sync_clock(const ADC_TypeDef* adc, const uint32_t ckmode) {
    writeField(adc_common(adc)->CCR, 3U, 16U, ckmode);
}

void adc_disable(ADC_TypeDef* adc) { clearMask(adc->CR, ADC_CR_ADEN | ADC_CR_ADSTART); }

void adc_init_triggered(ADC_TypeDef* adc, const uint32_t channel, const uint32_t extsel,
                        const uint32_t smp, const AdcTriggerEdge edge) {
    delayMicroseconds(20);   // tADCVREG_STUP, as the real driver waits
    adc->IER = 0;
    adc->CFGR2 = 0;
    adc->CFGR = ADC_CFGR_DMAEN | ADC_CFGR_DMACFG | ADC_CFGR_OVRMOD | extsel << ADC_CFGR_EXTSEL_Pos |
                static_cast<uint32_t>(edge) << ADC_CFGR_EXTEN_Pos;
    adc->SQR1 = channel << ADC_SQR1_SQ1_Pos;
    if (channel < 10) writeField(adc->SMPR1, 7U, channel * 3, smp);
    else writeField(adc->SMPR2, 7U, (channel - 10) * 3, smp);
    setMask(adc->CR, ADC_CR_ADEN);
    setMask(adc->CR, ADC_CR_ADSTART);
}

// ---- DMA ---------------------------------------------------------------------------------------

void dma_reset(DmaChannel* ch) {
    const int i = dma_index(ch);
    if (i >= 0) dma[i] = {};
}

void dma_setup(DmaChannel* ch, const DmaDirection dir, const DmaWidth width,
               volatile void* periph_addr, volatile void* mem_addr, const uint32_t count,
               const uint32_t request, const bool circular) {
    const int i = dma_index(ch);
    if (i < 0) return;
    dma[i] = {true, false, dir, width, periph_addr, mem_addr, count, 0, request, circular};
}

void dma_enable(DmaChannel* ch) {
    const int i = dma_index(ch);
    if (i >= 0 && dma[i].configured) dma[i].enabled = true;
}

void dma_enable_events(DmaChannel* /*ch*/, uint32_t /*events*/) {}

uint32_t dma_remaining(const DmaChannel* ch) {
    const int i = dma_index(ch);
    if (i < 0) return 0;
    return dma[i].count - dma[i].pos;
}

uint32_t dma_take_events(DmaChannel* /*ch*/) { return 0; }

IRQn_Type dma_irqn(const DmaChannel* ch) {
    const int i = dma_index(ch);
    return static_cast<IRQn_Type>(GPDMA1_Channel0_IRQn + (i < 0 ? 0 : i));
}

}  // extern "C"

namespace sim::io {

bool adc_trigger(ADC_TypeDef* adc, const int trigger_id, const bool rising, const PinName pin,
                 const uint16_t value) {
    if (adc == nullptr || trigger_id < 0) return false;
    if ((adc->CR & (ADC_CR_ADEN | ADC_CR_ADSTART)) != (ADC_CR_ADEN | ADC_CR_ADSTART)) return false;

    const uint32_t extsel = (adc->CFGR & ADC_CFGR_EXTSEL_Msk) >> ADC_CFGR_EXTSEL_Pos;
    const uint32_t exten = (adc->CFGR >> ADC_CFGR_EXTEN_Pos) & 3U;
    if (extsel != static_cast<uint32_t>(trigger_id)) return false;
    if (!((rising && (exten & ADC_TRIGGER_RISING)) || (!rising && (exten & ADC_TRIGGER_FALLING))))
        return false;

    // The pin must be in analog mode and be what SQ1 converts.
    if (pin_mode(pin) != MODE_ANALOG) return false;
    const uint32_t sq1 = (adc->SQR1 >> ADC_SQR1_SQ1_Pos) & 0x1FU;
    if (pinmap_peripheral(pin, PinMap_ADC) != adc ||
        STM_PIN_CHANNEL(pinmap_function(pin, PinMap_ADC)) != sq1)
        return false;

    adc->DR = value & 0xFFFU;
    if (!(adc->CFGR & ADC_CFGR_DMAEN)) return true;

    for (DmaState& d : dma) {
        if (!d.enabled || d.request != adc_request(adc) || d.periph != &adc->DR) continue;
        if (d.pos >= d.count) {
            if (!d.circular) break;
            d.pos = 0;
        }
        switch (d.width) {
            case DMA_WIDTH_8: static_cast<volatile uint8_t*>(d.mem)[d.pos] = static_cast<uint8_t>(adc->DR); break;
            case DMA_WIDTH_16: static_cast<volatile uint16_t*>(d.mem)[d.pos] = static_cast<uint16_t>(adc->DR); break;
            case DMA_WIDTH_32: static_cast<volatile uint32_t*>(d.mem)[d.pos] = adc->DR; break;
        }
        if (++d.pos >= d.count && d.circular) d.pos = 0;
        break;
    }
    return true;
}

}  // namespace sim::io
