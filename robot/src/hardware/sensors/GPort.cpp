#include "hardware/sensors/GPort.h"

#include <PeripheralPins.h>
#include <pinmap.h>

#include "hardware/io/adc/ADC.h"
#include "hardware/io/gpio/pwm.h"
#include "hardware/io/timer/LPTIM.h"
#include "hardware/io/timer/Timer.h"
#include "optimizations/bitboard.h"

constexpr uint8_t GPORT_RESET_PRIORITY = 2;
constexpr uint32_t MMS_OC1REF = 4;   // TRGO = OCxREF: 4 + channel index

bool GPort::begin(const GPortHardware& hw, const GSensorKind kind) {
    return begin(hw, kind, kind == GSensorKind::Line ? G_TIMING_LINE : G_TIMING_IR);
}

bool GPort::begin(const GPortHardware& hw, const GSensorKind kind, const GPortTiming& timing) {
    sensorKind = kind;
    if (kind == GSensorKind::None) return true;

    frameLen  = kind == GSensorKind::Line ? MAX_FRAME : SENSORS;
    bufferLen = 2 * frameLen;   // a published frame stays intact for one more loop
    dma       = hw.dma;
    synced    = false;
    frameSeq  = 0;
    desyncs   = 0;

    // ---- ADC + DMA: stream one conversion per clock pulse ----
    auto* adc = static_cast<ADC_TypeDef*>(pinmap_peripheral(hw.adcPin, PinMap_ADC));
    if (adc == nullptr) return false;
    const uint32_t channel = STM_PIN_CHANNEL(pinmap_function(hw.adcPin, PinMap_ADC));

    adc_clock_enable(adc);
    adc_disable(adc);
    adc_set_sync_clock(adc, 3U);   // HCLK / 4
    pinmap_pinout(hw.adcPin, PinMap_ADC);

    dma_setup(dma, DMA_PERIPH_TO_MEM, DMA_WIDTH_16, &adc->DR, buffer, bufferLen, hw.dmaRequest, true);
    dma_enable(dma);

    const AdcTriggerEdge sampleEdge = hw.muxAdvanceEdge == GClockEdge::Falling
                                    ? ADC_TRIGGER_RISING : ADC_TRIGGER_FALLING;
    adc_init_triggered(adc, channel, ADC_EXTSEL_FROM_HAL(hw.adcTrigger), timing.adcSampleTime, sampleEdge);

    // ---- Loop boundary ----
    if (!exti_attach(hw.resetPin, hw.resetEdge, onReset, this, GPORT_RESET_PRIORITY))
        return false;

    // ---- Board outputs ----
    if (kind == GSensorKind::Line && timing.modulationHz != 0) {
        if (!startModulation(hw, timing.modulationHz)) return false;
    } else if (hw.modulationPin != NC) {
        const PinName pad = static_cast<PinName>(hw.modulationPin & PNAME_MASK);
        pinMode(pinNametoDigitalPin(pad), OUTPUT);
        digitalWriteFast(pad, LOW);
    }

    return startClock(hw, timing.sampleRateHz);
}

// 50% duty clock whose OCxREF (TIMx) or channel output (LPTIMx) also triggers the ADC.
bool GPort::startClock(const GPortHardware& hw, const uint32_t rateHz) {
    if (rateHz == 0) return false;

#if defined(MCU_FAMILY_H5)
    if (hw.clockLptim != nullptr) {
        LptimPwmInfo info;
        if (!lptim_pwm_start(hw.clockLptim, hw.clockLptimChannel, rateHz, 500, false, &info))
            return false;
        lptim_pin_connect(hw.clockPin, hw.clockLptimAf);
        return true;
    }
#endif

    uint8_t ch = 0;
    bool complementary = false;
    TIM_TypeDef* tim = tim_from_pin(hw.clockPin, &ch, &complementary);
    if (tim == nullptr) return false;

    tim_clock_enable(tim);
    tim_stop(tim);

    const uint32_t clk = tim_clock_hz(tim);
    const uint32_t periodTicks = clk / rateHz;
    const uint32_t psc = (periodTicks - 1U) / 0x10000U;
    const uint32_t arr = clk / ((psc + 1U) * rateHz) - 1U;

    tim->PSC = psc;
    tim->ARR = arr;

    volatile uint32_t* ccmr = &tim->CCMR1 + (ch >> 1);
    writeField(*ccmr, 0xFFU, (ch & 1U) * 8U, 0x68U);   // PWM mode 1, preload
    (&tim->CCR1)[ch] = (arr + 1U) / 2U;

    setBit(tim->CCER, ch * 4U + (complementary ? 2U : 0U));
    if (complementary) setBit(tim->CCER, ch * 4U + 3U);
    if (IS_TIM_BREAK_INSTANCE(tim)) setMask(tim->BDTR, TIM_BDTR_MOE);

    tim_set_trgo(tim, MMS_OC1REF + ch);
    setMask(tim->CR1, TIM_CR1_ARPE);
    tim->EGR = TIM_EGR_UG;
    tim->SR = 0;

    tim_pin_connect(hw.clockPin);
    tim_start(tim);
    return true;
}

bool GPort::startModulation(const GPortHardware& hw, const uint32_t freqHz) {
    PwmPin pw = pwm_pin_init(hw.modulationPin);
    if (pw.timer == nullptr) return false;

    tim_clock_enable(pw.timer);
    const uint32_t resolution = tim_clock_hz(pw.timer) / freqHz - 1U;   // PSC = 0
    if (resolution > 0xFFFFU) return false;

    pwm_init(&pw, freqHz, resolution);
    pwm_write(&pw, (resolution + 1U) / 2U);
    return true;
}

void GPort::onReset(void* ctx) {
    GPort& p = *static_cast<GPort*>(ctx);
    const uint32_t pos = (p.bufferLen - dma_remaining(p.dma)) % p.bufferLen;

    if (p.synced) {
        const uint32_t samples = (pos + p.bufferLen - p.lastResetPos) % p.bufferLen;
        if (samples == p.frameLen) {
            p.frameEnd = pos;
            ++p.frameSeq;
        } else {
            ++p.desyncs;
        }
    }
    p.synced = true;
    p.lastResetPos = pos;
}

void GPort::setLineOrder(const LineColor order[4]) {
    for (uint8_t i = 0; i < 4; i++) lineOrder[i] = order[i];
}

bool GPort::readFrame(uint16_t* out) const {
    while (true) {
        const uint32_t seq = frameSeq;
        if (seq == 0) return false;

        uint32_t idx = (frameEnd + bufferLen - frameLen) % bufferLen;
        for (uint32_t i = 0; i < frameLen; i++) {
            out[i] = buffer[idx];
            if (++idx == bufferLen) idx = 0;
        }

        // DMA only overwrites this frame once the next loop is complete, which
        // also bumps the sequence: retry in that case.
        if (frameSeq == seq) return true;
    }
}

bool GPort::readIR(uint16_t out[SENSORS]) const {
    return sensorKind == GSensorKind::IR && readFrame(out);
}

bool GPort::readLine(LineFrame& out) const {
    if (sensorKind != GSensorKind::Line) return false;

    uint16_t raw[MAX_FRAME];
    if (!readFrame(raw)) return false;

    for (uint8_t sweep = 0; sweep < 4; sweep++) {
        const auto color = static_cast<uint8_t>(lineOrder[sweep]);
        for (uint8_t s = 0; s < SENSORS; s++)
            out.value[color][s] = raw[sweep * SENSORS + s];
    }
    return true;
}
