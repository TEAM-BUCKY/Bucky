#ifndef BUCKY_GPORT_H
#define BUCKY_GPORT_H

#include <Arduino.h>

#include "hardware/io/mcu.h"
#include "hardware/io/dma/DMA.h"
#include "hardware/io/gpio/gpio.h"
#include "hardware/io/irq/IRQ.h"

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

struct GPortHardware {
    PinName resetPin;
    ExtiEdge resetEdge;

    PinName clockPin;
    LPTIM_TypeDef* clockLptim;
    uint8_t clockLptimChannel;
    uint8_t clockLptimAf;
    uint32_t adcTrigger;
    GClockEdge muxAdvanceEdge;

    PinName adcPin;
    DmaChannel* dma;
    uint32_t dmaRequest;

    PinName modulationPin;
};

struct GPortTiming {
    uint32_t sampleRateHz;
    uint32_t modulationHz;
    uint8_t adcSampleTime;
};

// IR: the ball repeats its 8-pulse burst every 833.33 us, so each sensor gets
// at least two bursts (1666.67 us) before the next one is read.
constexpr GPortTiming G_TIMING_IR = {600, 0, 4};

// Line: 4000 samples per second, 44 kHz modulation, 4 cycles of ADC sample time.
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

    void setLineOrder(const LineColor order[4]);

    [[nodiscard]] GSensorKind kind() const { return sensorKind; }
    [[nodiscard]] uint32_t frameLength() const { return frameLen; }

    [[nodiscard]] uint32_t frameSequence() const { return frameSeq; }
    [[nodiscard]] bool hasNewFrame(const uint32_t lastSequence) const { return frameSeq != lastSequence; }

    [[nodiscard]] uint32_t desyncCount() const { return desyncs; }
    [[nodiscard]] uint32_t resetCount() const { return resets; }
    [[nodiscard]] LineColor lineColorAt(const uint8_t sweep) const { return lineOrder[sweep]; }

    // Manual clocking: take the clock pin from its timer and step the mux by
    // hand (e.g. to hold the line LEDs on a colour). Frames stop while held.
    bool holdClock();
    void stepClock() const;
    bool releaseClock();

    bool readIR(uint16_t out[SENSORS]) const;
    bool readLine(LineFrame& out) const;

private:
    static void onReset(void* ctx);

    static bool startClock(const GPortHardware& hw, uint32_t rateHz);

    GSensorKind sensorKind = GSensorKind::None;
    const GPortHardware* hardware = nullptr;
    GPortTiming portTiming = {};
    GpioPin clockGpio = {};
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
    volatile uint32_t resets = 0;
    volatile bool clockHeld = false;

    bool readFrame(uint16_t* out) const;
};

#endif // BUCKY_GPORT_H
