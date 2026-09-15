#ifndef BUCKY_ENCODER_H
#define BUCKY_ENCODER_H

#include <Arduino.h>
#include "io/gpio/gpio.h"
#include "optimizations/optimizations.h"

#define ENCODER_MAX 3

struct EncoderPins {
    int pinA;
    int pinB;
};

struct EncoderState {
    GpioPin gpioA;
    GpioPin gpioB;
    volatile int32_t ticks;
    volatile uint8_t lastA;
    volatile uint8_t lastB;
    int32_t prevTicks;
    uint32_t prevMicros;
    float speedTicksPerSec;
    bool active;
    bool hasInterruptA;
    bool hasInterruptB;
};

void encoder_init(uint8_t index, const EncoderPins& pins);

int32_t encoder_get_ticks(uint8_t index);
float   encoder_get_speed(uint8_t index);
void    encoder_reset(uint8_t index);
bool    encoder_is_active(uint8_t index);
void    encoder_update_speed(uint8_t index);

#endif // BUCKY_ENCODER_H
