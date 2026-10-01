#include "Encoder.h"

#include "hardware/io/irq/IRQ.h"
#include "hardware/io/timer/Timer.h"

static EncoderState encoders[ENCODER_MAX];

constexpr uint8_t ENCODER_EXTI_PRIORITY = 1;

// Input filter on TI1/TI2: fSAMPLING = fCK_INT / 4, N = 6. Rejects glitches of
// roughly 24 counter-clock periods (~100 ns at 250 MHz, ~140 ns at 170 MHz).
constexpr uint32_t ENCODER_INPUT_FILTER = 0x6U;

// ---------------------------------------------------------------- TIMER ---

static bool try_init_timer(EncoderState& e, const EncoderPins& pins) {
    uint8_t chA = 0, chB = 0;
    TIM_TypeDef* timA = tim_from_pin(pins.pinA, &chA, nullptr);
    TIM_TypeDef* timB = tim_from_pin(pins.pinB, &chB, nullptr);

    if (timA == nullptr || timA != timB || chA != 0 || chB != 1)
        return false;
    if (!IS_TIM_ENCODER_INTERFACE_INSTANCE(timA))
        return false;

    TIM_TypeDef* tim = timA;
    tim_clock_enable(tim);
    tim_stop(tim);

    tim->PSC = 0;
    tim->ARR = 0xFFFFU;   // 16-bit wrap on every timer keeps the delta math uniform

    // CC1S = CC2S = 01: IC1 on TI1, IC2 on TI2, both filtered.
    tim->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0
               | ENCODER_INPUT_FILTER << TIM_CCMR1_IC1F_Pos
               | ENCODER_INPUT_FILTER << TIM_CCMR1_IC2F_Pos;
    tim->CCER = 0;        // non-inverted inputs

    // SMS = 001: encoder mode 1, count on TI1 edges, direction from TI2 level.
    // Up when TI1 rises with TI2 low, matching the previous EXTI decoder's sign.
    tim->SMCR = (tim->SMCR & ~TIM_SMCR_SMS_Msk) | TIM_SMCR_SMS_0;

    tim->CNT = 0;
    tim->EGR = TIM_EGR_UG;
    tim->SR  = 0;

    tim_pin_connect(pins.pinA);
    tim_pin_connect(pins.pinB);
    tim_start(tim);

    e.backend   = EncoderBackend::Timer;
    e.timer     = tim;
    e.lastCount = 0;
    return true;
}

// Fold the hardware counter into the 32-bit tick total. Needs to run at least
// once per 32767 ticks, which every speed update easily does.
static FORCE_INLINE void sample_timer(EncoderState& e) {
    const auto count = static_cast<uint16_t>(e.timer->CNT);
    const auto delta = static_cast<int16_t>(count - e.lastCount);
    e.lastCount = count;
    if (delta != 0) {
        e.ticks += delta;
        e.active = true;
    }
}

// ----------------------------------------------------------------- EXTI ---

static void encoder_exti_isr(void* ctx) {
    EncoderState& e = *static_cast<EncoderState*>(ctx);
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

static void init_exti(EncoderState& e, const EncoderPins& pins) {
    e.gpioA = gpio_pin_init(pins.pinA);
    e.gpioB = gpio_pin_init(pins.pinB);
    gpio_mode(e.gpioA, INPUT_PULLUP);
    gpio_mode(e.gpioB, INPUT_PULLUP);

    e.lastA = gpio_read(e.gpioA);
    e.lastB = gpio_read(e.gpioB);

    // Prefer A as the edge-counting pin; fall back to B only if A's EXTI line
    // is already claimed by another encoder. Never both — see encoder_exti_isr().
    e.hasInterruptA = exti_attach(pins.pinA, EXTI_BOTH, encoder_exti_isr, &e, ENCODER_EXTI_PRIORITY);
    e.hasInterruptB = !e.hasInterruptA
                   && exti_attach(pins.pinB, EXTI_BOTH, encoder_exti_isr, &e, ENCODER_EXTI_PRIORITY);

    e.backend = (e.hasInterruptA || e.hasInterruptB) ? EncoderBackend::Exti : EncoderBackend::None;
}

// ------------------------------------------------------------------ API ---

void encoder_init(const uint8_t index, const EncoderPins& pins) {
    if (index >= ENCODER_MAX) return;

    EncoderState& e = encoders[index];
    e = EncoderState{};
    e.prevMicros = micros();

    if (!try_init_timer(e, pins))
        init_exti(e, pins);
}

int32_t encoder_get_ticks(const uint8_t index) {
    EncoderState& e = encoders[index];
    if (e.backend == EncoderBackend::Timer) {
        sample_timer(e);
        return e.ticks;
    }
    __disable_irq();
    const int32_t t = e.ticks;
    __enable_irq();
    return t;
}

float encoder_get_speed(const uint8_t index) {
    return encoders[index].speedTicksPerSec;
}

void encoder_reset(const uint8_t index) {
    EncoderState& e = encoders[index];
    __disable_irq();
    if (e.backend == EncoderBackend::Timer)
        e.lastCount = static_cast<uint16_t>(e.timer->CNT);
    e.ticks = 0;
    e.prevTicks = 0;
    e.speedTicksPerSec = 0.0f;
    __enable_irq();
}

bool encoder_is_active(const uint8_t index) {
    EncoderState& e = encoders[index];
    if (e.backend == EncoderBackend::Timer) sample_timer(e);
    return e.active;
}

EncoderBackend encoder_backend(const uint8_t index) {
    return encoders[index].backend;
}

void encoder_update_speed(const uint8_t index) {
    const uint32_t now = micros();
    EncoderState& e = encoders[index];

    const uint32_t dt = now - e.prevMicros;
    if (dt < 10000) return;   // accumulate at least 10 ms before computing speed

    const int32_t ticks = encoder_get_ticks(index);

    const int32_t delta = ticks - e.prevTicks;
    e.prevTicks = ticks;
    e.prevMicros = now;

    const float instantSpeed = static_cast<float>(delta) * 1000000.0f / static_cast<float>(dt);

    constexpr float kEmaAlpha = 0.3f;
    constexpr float kEmaBeta  = 1.0f - kEmaAlpha;
    e.speedTicksPerSec = e.speedTicksPerSec * kEmaBeta + instantSpeed * kEmaAlpha;
}
