#include "Encoder.h"

static EncoderState encoders[ENCODER_MAX];

// Track which EXTI lines are already claimed so that pins sharing an
// EXTI line (e.g. PB2 and PA2 both use EXTI2) don't overwrite each other.
static uint16_t extiUsedMask = 0;

static void encoder_isr(const uint8_t idx) {
    EncoderState& e = encoders[idx];
    const uint8_t a = gpio_read(e.gpioA);
    const uint8_t b = gpio_read(e.gpioB);

    // Count edges on whichever pin owns the EXTI line for this encoder. The
    // other pin is sampled to decode direction. This keeps every encoder at a
    // fixed 2x quadrature resolution and means each motor only needs one EXTI
    // line — so no two encoders can collide on the same line (e.g. PA2/PB2
    // both on EXTI2).
    if (e.hasInterruptA && a != e.lastA)
        e.ticks += (a == b) ? -1 : 1;
    else if (e.hasInterruptB && b != e.lastB)
        e.ticks += (b == a) ? 1 : -1;

    e.lastA = a;
    e.lastB = b;
    e.active = true;
}

static void encoder_isr0() { encoder_isr(0); }
static void encoder_isr1() { encoder_isr(1); }
static void encoder_isr2() { encoder_isr(2); }

static constexpr void (*const isrTable[ENCODER_MAX])() = {
    encoder_isr0, encoder_isr1, encoder_isr2
};

static bool tryAttachInterrupt(int pin, uint8_t encoderIndex) {
    const int irq = digitalPinToInterrupt(pin);
    if (irq < 0) return false;

    // Use the actual EXTI line number (pin index within the port, 0-15),
    // NOT the Arduino pin number which can be >= 16 and overflow the mask.
    const uint8_t extiLine = STM_PIN(digitalPinToPinName(pin));
    const uint16_t bit = 1U << extiLine;
    if (extiUsedMask & bit) return false;   // EXTI line already taken

    attachInterrupt(irq, isrTable[encoderIndex], CHANGE);
    extiUsedMask |= bit;
    return true;
}

void encoder_init(const uint8_t index, const EncoderPins& pins) {
    if (index >= ENCODER_MAX) return;

    EncoderState& e = encoders[index];
    e.gpioA = gpio_pin_init(pins.pinA);
    e.gpioB = gpio_pin_init(pins.pinB);
    gpio_mode(e.gpioA, INPUT_PULLUP);
    gpio_mode(e.gpioB, INPUT_PULLUP);

    e.ticks = 0;
    e.prevTicks = 0;
    e.prevMicros = micros();
    e.speedTicksPerSec = 0.0f;
    e.active = false;

    e.lastA = gpio_read(e.gpioA);
    e.lastB = gpio_read(e.gpioB);

    // Prefer A as the edge-counting pin; fall back to B only if A's EXTI line
    // is already claimed by another encoder. Never both — see encoder_isr().
    e.hasInterruptA = tryAttachInterrupt(pins.pinA, index);
    e.hasInterruptB = !e.hasInterruptA && tryAttachInterrupt(pins.pinB, index);
}

int32_t encoder_get_ticks(const uint8_t index) {
    __disable_irq();
    const int32_t t = encoders[index].ticks;
    __enable_irq();
    return t;
}

float encoder_get_speed(const uint8_t index) {
    __disable_irq();
    const float s = encoders[index].speedTicksPerSec;
    __enable_irq();
    return s;
}

void encoder_reset(const uint8_t index) {
    __disable_irq();
    encoders[index].ticks = 0;
    encoders[index].prevTicks = 0;
    encoders[index].speedTicksPerSec = 0.0f;
    __enable_irq();
}

bool encoder_is_active(const uint8_t index) {
    return encoders[index].active;
}

void encoder_update_speed(const uint8_t index) {
    const uint32_t now = micros();
    EncoderState& e = encoders[index];

    const uint32_t dt = now - e.prevMicros;
    if (dt < 10000) return;   // accumulate at least 10 ms before computing speed

    __disable_irq();
    const int32_t ticks = e.ticks;
    __enable_irq();

    const int32_t delta = ticks - e.prevTicks;
    e.prevTicks = ticks;
    e.prevMicros = now;

    const float instantSpeed = static_cast<float>(delta) * 1000000.0f / static_cast<float>(dt);

    constexpr float alpha = 0.3f;
    e.speedTicksPerSec = e.speedTicksPerSec * (1.0f - alpha) + instantSpeed * alpha;
}
