#ifndef BUCKY_GPORT_H
#define BUCKY_GPORT_H

#include <Arduino.h>

#include "hardware/io/mcu.h"
#include "hardware/io/dma/DMA.h"
#include "hardware/io/irq/IRQ.h"

// Generalised sensor port ("G" port): one connector that serves both the IR
// ring and the line sensor, which share the same analog mux + ADC principle.
//
//   G_CLK  clock pulses; every pulse steps the board's mux to the next sensor
//   G_ADC  analog output of the selected sensor (0..3.3 V)
//   G_RST  pulse from the board after a complete loop (input, EXTI)
//   G_MOD  44 kHz LED modulation, line sensor only
//
// Acquisition is interrupt-free per sample: the clock timer's output doubles
// as the ADC trigger, so each pulse produces one conversion that DMA streams
// into a circular buffer. The ADC samples on the clock edge opposite to the
// one that advances the mux, i.e. half a clock period after switching.
//
// G_RST is the only per-loop interrupt. It records where the DMA write pointer
// is, which marks the end of a complete frame; readers copy that frame out.
//
//   IR:   16 samples per loop, sensor 0..15.
//   Line: 64 samples per loop, four 16-sensor sweeps in lineOrder (red, green,
//         blue, dark by default).

enum class GSensorKind : uint8_t {
    None,
    IR,
    Line,
};

enum class LineColor : uint8_t {
    Red,
    Green,
    Blue,
    Dark,
};

enum class GClockEdge : uint8_t {
    Rising,
    Falling,
};

// Board wiring of one G port.
struct GPortHardware {
    PinName resetPin;
    ExtiEdge resetEdge;          // edge of the G_RST pulse that marks the loop end

    PinName clockPin;
    // Clock on TIMx: pin must be in the core pin map, adcTrigger = its TIMx_TRGO.
    // Clock on LPTIMx: set clockLptim; the pin is not in the core pin map, so
    // its alternate function is given here. adcTrigger = LPTIMx_CH1.
    LPTIM_TypeDef* clockLptim;
    uint8_t clockLptimChannel;
    uint8_t clockLptimAf;
    uint32_t adcTrigger;         // HAL ADC_EXTERNALTRIG_*
    GClockEdge muxAdvanceEdge;   // the ADC samples on the other edge

    PinName adcPin;              // _ALTn picks the ADC instance (PC_0_ALT1 = ADC2)
    DmaChannel* dma;
    uint32_t dmaRequest;

    PinName modulationPin;
};

struct GPortTiming {
    uint32_t sampleRateHz;       // clock pulses per second
    uint32_t modulationHz;       // 0 = G_MOD held low
    uint8_t adcSampleTime;       // SMPR code: 4 = 47.5 cycles
};

// IR: the ball repeats its 8-pulse burst every 833.33 us, so each sensor gets
// at least two bursts (1666.67 us) before the next one is read.
constexpr GPortTiming G_TIMING_IR = {600, 0, 4};

// Line: no timing requirement given yet; 250 us per sample (11 carrier periods
// at 44 kHz), 16 ms per 64-sample loop. Tune on the hardware.
constexpr GPortTiming G_TIMING_LINE = {4000, 44000, 4};

struct LineFrame {
    uint16_t value[4][16];   // [LineColor][sensor]

    [[nodiscard]] uint16_t get(LineColor color, uint8_t sensor) const {
        return value[static_cast<uint8_t>(color)][sensor];
    }
};

class GPort {
public:
    static constexpr uint32_t SENSORS = 16;
    static constexpr uint32_t MAX_FRAME = 4 * SENSORS;

    bool begin(const GPortHardware& hw, GSensorKind kind, const GPortTiming& timing);
    bool begin(const GPortHardware& hw, GSensorKind kind);

    // Line sensor sweep order within one loop.
    void setLineOrder(const LineColor order[4]);

    [[nodiscard]] GSensorKind kind() const { return sensorKind; }
    [[nodiscard]] uint32_t frameLength() const { return frameLen; }

    // Incremented for every complete, correctly sized loop.
    [[nodiscard]] uint32_t frameSequence() const { return frameSeq; }
    [[nodiscard]] bool hasNewFrame(uint32_t lastSequence) const { return frameSeq != lastSequence; }

    // Loops whose sample count did not match the frame length (missed or
    // spurious G_RST pulse). Frames resume after two good resets.
    [[nodiscard]] uint32_t desyncCount() const { return desyncs; }

    // Copy the latest complete loop, first sample first. False before the first frame.
    bool readFrame(uint16_t* out) const;

    bool readIR(uint16_t out[SENSORS]) const;
    bool readLine(LineFrame& out) const;

private:
    static void onReset(void* ctx);

    static bool startClock(const GPortHardware& hw, uint32_t rateHz);

    GSensorKind sensorKind = GSensorKind::None;
    DmaChannel* dma = nullptr;
    uint32_t frameLen = 0;
    uint32_t bufferLen = 0;
    uint16_t buffer[2 * MAX_FRAME] = {};
    LineColor lineOrder[4] = {LineColor::Red, LineColor::Green, LineColor::Blue, LineColor::Dark};

    volatile bool synced = false;
    volatile uint32_t lastResetPos = 0;
    volatile uint32_t frameEnd = 0;
    volatile uint32_t frameSeq = 0;
    volatile uint32_t desyncs = 0;
};

#endif // BUCKY_GPORT_H
