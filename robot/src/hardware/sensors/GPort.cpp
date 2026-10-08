#include "hardware/sensors/GPort.h"

#include <PeripheralPins.h>
#include <pinmap.h>

#include "hardware/io/adc/ADC.h"
#include "hardware/io/gpio/gpio.h"
#include "hardware/io/timer/LPTIM.h"
#include "hardware/io/timer/Timer.h"

constexpr uint8_t GPORT_RESET_PRIORITY = 2;

bool GPort::begin(const GPortHardware& hw, const GSensorKind kind) {
    return begin(hw, kind, kind == GSensorKind::Line ? G_TIMING_LINE : G_TIMING_IR);
}

bool GPort::begin(const GPortHardware& hw, const GSensorKind kind, const GPortTiming& timing) {
    sensorKind = kind;
    if (kind == GSensorKind::None) return true;

    hardware   = &hw;
    portTiming = timing;
    clockHeld  = false;

    frameLen  = kind == GSensorKind::Line ? MAX_FRAME : SENSORS;
    bufferLen = 2 * frameLen;   // A frame stays intact for one more loop
    dma       = hw.dma;
    synced    = false;
    frameSeq  = 0;
    desyncs   = 0;

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

    const GpioPin resetGpio = gpio_pin_init(hw.resetPin);
    gpio_mode(resetGpio, INPUT_PULLDOWN);
    gpio_write(resetGpio, 0);
    delay(1);
    gpio_write(resetGpio, 1);
    delay(1);
    gpio_write(resetGpio, 0);

    if (!exti_attach(hw.resetPin, hw.resetEdge, onReset, this, GPORT_RESET_PRIORITY))
        return false;

    if (kind == GSensorKind::Line && timing.modulationHz != 0) {
        TIM_TypeDef* mod = tim_pwm_setup(hw.modulationPin, timing.modulationHz, 500, nullptr);
        if (mod == nullptr) return false;
        tim_start(mod);
    } else if (hw.modulationPin != NC) {
        gpio_hold_low(hw.modulationPin);
    }

    return startClock(hw, timing.sampleRateHz);
}

bool GPort::startClock(const GPortHardware& hw, const uint32_t rateHz) {
#if defined(MCU_FAMILY_H5)
    if (hw.clockLptim != nullptr) {
        if (!lptim_pwm_start(hw.clockLptim, hw.clockLptimChannel, rateHz, 500))
            return false;
        lptim_pin_connect(hw.clockPin, hw.clockLptimAf);
        return true;
    }
#endif

    uint8_t ch = 0;
    TIM_TypeDef* tim = tim_pwm_setup(hw.clockPin, rateHz, 500, &ch);
    if (tim == nullptr) return false;

    tim_set_trgo(tim, TIM_TRGO_OCREF(ch));
    tim_start(tim);
    return true;
}

void GPort::onReset(void* ctx) {
    GPort& p = *static_cast<GPort*>(ctx);
    ++p.resets;
    if (p.clockHeld) {       // stepped by hand: the DMA position means nothing
        p.synced = false;
        return;
    }

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

bool GPort::holdClock() {
    if (hardware == nullptr || clockHeld) return false;
    clockHeld = true;
    clockGpio = gpio_pin_init(hardware->clockPin);
    gpio_low(clockGpio);
    gpio_mode(clockGpio, OUTPUT);
    return true;
}

// One high pulse: the mux advances on whichever of its two edges it uses.
void GPort::stepClock() const {
    gpio_high(clockGpio);
    delayMicroseconds(1);
    gpio_low(clockGpio);
    delayMicroseconds(1);
}

bool GPort::releaseClock() {
    if (!clockHeld) return false;
    synced    = false;
    clockHeld = false;
    return startClock(*hardware, portTiming.sampleRateHz);
}

void GPort::setLineOrder(const LineColor order[4]) {
    for (uint8_t i = 0; i < 4; i++) lineOrder[i] = order[i];
}

// Due to design constraints the first 8 channels were flipped.
// This is for both the Line Sensor Controller boards and the IR Sensor Controller boards.
// Every colour sweep goes through the same mux, so the flip repeats each SENSORS samples.
uint32_t translatePosition(const uint32_t pos) {
    const uint32_t ch = pos % GPort::SENSORS;
    return ch < 8 ? pos - ch + (7 - ch) : pos;
}

bool GPort::readFrame(uint16_t* out) const {
    while (true) {
        const uint32_t seq = frameSeq;
        if (seq == 0) return false;

        uint32_t idx = (frameEnd + bufferLen - frameLen) % bufferLen;
        for (uint32_t i = 0; i < frameLen; i++) {
            out[translatePosition(i)] = buffer[idx];
            if (++idx == bufferLen) idx = 0;
        }

        if (frameSeq == seq) return true;
    }
}

/**
 * Reads the IR sensor data
 * @param out Reference to an array where the sensor data will be stored. The array should have a size of SENSORS.
 * @return True if the sensor kind is IR and the frame was successfully read; false otherwise.
 */
bool GPort::readIR(uint16_t out[SENSORS]) const {
    return sensorKind == GSensorKind::IR && readFrame(out);
}

/**
 * Reads the line sensor data
 * @param out Reference to a LineFrame structure where the sensor data will be stored.
 * @return True if the sensor kind is Line and the frame was successfully read; false otherwise.
 */
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
