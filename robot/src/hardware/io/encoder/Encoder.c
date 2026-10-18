#include "Encoder.h"

#include <Arduino.h>

#include <hardware/io/timer/Timer.h>

#define ENCODER_INPUT_FILTER 0x6U

typedef struct EncoderState {
    TIM_TypeDef* timer;
    uint16_t lastCount;

    volatile int32_t ticks;
    int32_t prevTicks;
    uint32_t prevMicros;
    float speedTicksPerSec;
    bool active;
} EncoderState;

static EncoderState encoders[ENCODER_MAX];

static bool try_init_timer(EncoderState* e, const EncoderPins* pins) {
    uint8_t chA = 0, chB = 0;
    TIM_TypeDef* timA = tim_from_pin(pins->pinA, &chA, NULL);
    TIM_TypeDef* timB = tim_from_pin(pins->pinB, &chB, NULL);

    if (timA == NULL || timA != timB || chA != 0 || chB != 1)
        return false;
    if (!IS_TIM_ENCODER_INTERFACE_INSTANCE(timA))
        return false;

    TIM_TypeDef* tim = timA;
    tim_clock_enable(tim);
    tim_stop(tim);

    tim->PSC = 0;
    tim->ARR = 0xFFFFU;

    tim->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0
               | ENCODER_INPUT_FILTER << TIM_CCMR1_IC1F_Pos
               | ENCODER_INPUT_FILTER << TIM_CCMR1_IC2F_Pos;
    tim->CCER = 0;        // non-inverted inputs

    tim->SMCR = (tim->SMCR & ~TIM_SMCR_SMS_Msk) | TIM_SMCR_SMS_0;

    tim->CNT = 0;
    tim_load(tim);

    tim_pin_connect(pins->pinA);
    tim_pin_connect(pins->pinB);
    tim_start(tim);

    e->timer     = tim;
    e->lastCount = 0;

    return true;
}

static FORCE_INLINE void sample_timer(EncoderState* e) {
    const uint16_t count = (uint16_t)e->timer->CNT;
    const int16_t delta = (int16_t)(count - e->lastCount);
    e->lastCount = count;
    if (delta != 0) {
        e->ticks += delta;
        e->active = true;
    }
}

void encoder_init(const uint8_t index, const EncoderPins* pins) {
    if (index >= ENCODER_MAX) return;

    EncoderState* e = &encoders[index];
    *e = (EncoderState){0};
    e->prevMicros = micros();

    try_init_timer(e, pins);
}

int32_t encoder_get_ticks(const uint8_t index) {
    EncoderState* e = &encoders[index];
    if (e->timer) sample_timer(e);
    return e->ticks;
}

float encoder_get_speed(const uint8_t index) {
    return encoders[index].speedTicksPerSec;
}

void encoder_reset(const uint8_t index) {
    EncoderState* e = &encoders[index];
    __disable_irq();
    if (e->timer)
        e->lastCount = (uint16_t)e->timer->CNT;
    e->ticks = 0;
    e->prevTicks = 0;
    e->speedTicksPerSec = 0.0f;
    __enable_irq();
}

bool encoder_is_active(const uint8_t index) {
    EncoderState* e = &encoders[index];
    if (e->timer) sample_timer(e);
    return e->active;
}

void encoder_update_speed(const uint8_t index) {
    const uint32_t now = micros();
    EncoderState* e = &encoders[index];

    const uint32_t dt = now - e->prevMicros;
    if (dt < 10000) return;   // accumulate at least 10 ms before computing speed

    const int32_t ticks = encoder_get_ticks(index);

    const int32_t delta = ticks - e->prevTicks;
    e->prevTicks = ticks;
    e->prevMicros = now;

    const float instantSpeed = (float)delta * 1000000.0f / (float)dt;

    const float emaAlpha = 0.3f;
    const float emaBeta  = 1.0f - emaAlpha;
    e->speedTicksPerSec = e->speedTicksPerSec * emaBeta + instantSpeed * emaAlpha;
}
