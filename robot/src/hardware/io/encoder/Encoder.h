#ifndef BUCKY_ENCODER_H
#define BUCKY_ENCODER_H

#include <Arduino.h>
#include "hardware/io/gpio/gpio.h"
#include "optimizations/optimizations.h"

#define ENCODER_MAX 3

// A/B pins may carry an _ALTn suffix to select the timer (e.g. PC_6_ALT1 = TIM8).
struct EncoderPins {
    PinName pinA;
    PinName pinB;
};

enum class EncoderBackend : uint8_t {
    None,
    Timer,   // hardware quadrature: A/B on CH1/CH2 of the same timer
    Exti,    // fallback for pads without a timer pair (G474 board)
};

struct EncoderState {
    EncoderBackend backend;

    // Timer backend
    TIM_TypeDef* timer;
    uint16_t lastCount;

    // Exti backend
    GpioPin gpioA;
    GpioPin gpioB;
    volatile uint8_t lastA;
    volatile uint8_t lastB;
    bool hasInterruptA;
    bool hasInterruptB;

    volatile int32_t ticks;
    int32_t prevTicks;
    uint32_t prevMicros;
    float speedTicksPerSec;
    bool active;
};

// Both backends count 2 ticks per encoder cycle (every edge of channel A), so
// speed calibration carries over between boards.
void encoder_init(uint8_t index, const EncoderPins& pins);

int32_t encoder_get_ticks(uint8_t index);
float   encoder_get_speed(uint8_t index);
void    encoder_reset(uint8_t index);
bool    encoder_is_active(uint8_t index);
void    encoder_update_speed(uint8_t index);
EncoderBackend encoder_backend(uint8_t index);

#endif // BUCKY_ENCODER_H
