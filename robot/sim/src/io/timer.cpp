// Replacement for hardware/io/timer/Timer.c (same algorithms; the RCC/HAL clock queries become
// the fixed H562 timer clock), plus the simulator's view of what a timer drives onto a pad.
#include "hardware/io/timer/Timer.h"

#include <Arduino.h>
#include <PeripheralPins.h>
#include <pinmap.h>

#include "io/io.h"

extern "C" {

void tim_clock_enable(TIM_TypeDef* /*tim*/) {}

uint32_t tim_clock_hz(const TIM_TypeDef* /*tim*/) {
    return static_cast<uint32_t>(sim::io::TIMER_CLOCK_HZ);
}

TIM_TypeDef* tim_from_pin(const PinName pin, uint8_t* channel, bool* complementary) {
    for (const PinMap* map = PinMap_TIM; map->pin != NC; map++) {
        if (map->pin != pin) continue;
        if (channel) *channel = static_cast<uint8_t>(STM_PIN_CHANNEL(map->function) - 1U);
        if (complementary) *complementary = STM_PIN_INVERTED(map->function) != 0U;
        return static_cast<TIM_TypeDef*>(map->peripheral);
    }
    return nullptr;
}

void tim_pin_connect(const PinName pin) { pinmap_pinout(pin, PinMap_TIM); }

uint32_t tim_set_frequency(TIM_TypeDef* tim, const uint32_t freq_hz) {
    if (freq_hz == 0U) return 0U;
    const uint32_t clk = tim_clock_hz(tim);
    const uint32_t ticks = clk / freq_hz;
    if (ticks < 2U) return 0U;
    const uint32_t psc = (ticks - 1U) / 0x10000U;
    if (psc > 0xFFFFU) return 0U;
    const uint32_t period = clk / ((psc + 1U) * freq_hz);
    tim->PSC = psc;
    tim->ARR = period - 1U;
    return period;
}

void tim_pwm_channel_enable(TIM_TypeDef* tim, const uint8_t channel, const bool complementary) {
    volatile uint32_t* ccmr = &tim->CCMR1 + (channel >> 1);
    writeField(*ccmr, 0xFFU, (channel & 1U) * 8U, 0x68U);
    setBit(tim->CCER, channel * 4U + (complementary ? 2U : 0U));
    if (complementary) setBit(tim->CCER, channel * 4U + 3U);
    if (IS_TIM_BREAK_INSTANCE(tim)) setMask(tim->BDTR, TIM_BDTR_MOE);
}

TIM_TypeDef* tim_pwm_setup(const PinName pin, const uint32_t freq_hz, const uint16_t duty_permille,
                           uint8_t* channel) {
    uint8_t ch = 0;
    bool complementary = false;
    TIM_TypeDef* tim = tim_from_pin(pin, &ch, &complementary);
    if (tim == nullptr) return nullptr;

    tim_clock_enable(tim);
    tim_stop(tim);

    const uint32_t period = tim_set_frequency(tim, freq_hz);
    if (period == 0U) return nullptr;

    const uint32_t duty = duty_permille > 1000U ? 1000U : duty_permille;
    *tim_ccr(tim, ch) = period * duty / 1000U;
    tim_pwm_channel_enable(tim, ch, complementary);
    setMask(tim->CR1, TIM_CR1_ARPE);
    tim_load(tim);

    tim_pin_connect(pin);
    if (channel) *channel = ch;
    return tim;
}

}  // extern "C"

namespace sim::io {

namespace {

bool same_pad(const PinName a, const PinName b) { return pad(a) == pad(b); }

// LPTIM outputs are not in the core pin map; the board header passes the AF number explicitly.
struct LptimPin {
    PinName pin;
    uint32_t af;
    LPTIM_TypeDef* lptim;
    uint8_t channel;
};
const LptimPin LPTIM_PINS[] = {
    {PB_2, 5, LPTIM1, 0},
};

}  // namespace

Waveform pad_waveform(const PinName p) {
    Waveform w;
    const uint32_t mode = pin_mode(p);
    if (mode == MODE_OUTPUT) {
        w.static_level = odr_level(p);
        return w;
    }
    if (mode != MODE_AF) return w;

    const uint32_t af = pin_af(p);
    for (const PinMap* map = PinMap_TIM; map->pin != NC; map++) {
        if (!same_pad(map->pin, p) || STM_PIN_AFNUM(map->function) != af) continue;
        auto* tim = static_cast<TIM_TypeDef*>(map->peripheral);
        const uint8_t ch = static_cast<uint8_t>(STM_PIN_CHANNEL(map->function) - 1U);
        const bool compl_ = STM_PIN_INVERTED(map->function) != 0U;
        w.tim = tim;
        w.channel = ch;
        const bool counting = (tim->CR1 & TIM_CR1_CEN) != 0;
        const bool enabled = (tim->CCER >> (ch * 4U + (compl_ ? 2U : 0U))) & 1U;
        const bool moe = !IS_TIM_BREAK_INSTANCE(tim) || (tim->BDTR & TIM_BDTR_MOE);
        if (!counting || !enabled || !moe) return w;
        const uint64_t arr1 = static_cast<uint64_t>(tim->ARR) + 1U;
        const uint64_t ccr = *tim_ccr(tim, ch);
        w.timer = true;
        w.period_ns = (static_cast<uint64_t>(tim->PSC) + 1U) * arr1 * 1'000'000'000ULL / TIMER_CLOCK_HZ;
        w.duty = ccr >= arr1 ? 1.0 : static_cast<double>(ccr) / static_cast<double>(arr1);
        return w;
    }
    for (const LptimPin& lp : LPTIM_PINS) {
        if (!same_pad(lp.pin, p) || lp.af != af) continue;
        LPTIM_TypeDef* t = lp.lptim;
        w.lptim = t;
        w.channel = lp.channel;
        const bool counting = (t->CR & LPTIM_CR_ENABLE) && (t->CR & LPTIM_CR_CNTSTRT);
        const uint32_t ccE = lp.channel == 0 ? LPTIM_CCMR1_CC1E : LPTIM_CCMR1_CC2E;
        if (!counting || !(t->CCMR1 & ccE)) return w;
        const uint32_t presc = (t->CFGR & LPTIM_CFGR_PRESC_Msk) >> LPTIM_CFGR_PRESC_Pos;
        const uint64_t arr1 = static_cast<uint64_t>(t->ARR) + 1U;
        const uint64_t ccr = lp.channel == 0 ? t->CCR1 : t->CCR2;
        w.timer = true;
        w.period_ns = (arr1 << presc) * 1'000'000'000ULL / LPTIM_CLOCK_HZ;
        w.duty = ccr >= arr1 ? 1.0 : static_cast<double>(ccr) / static_cast<double>(arr1);
        return w;
    }
    return w;
}

int adc_trigger_id(const Waveform& w) {
    if (!w.timer) return -1;
    if (w.tim == TIM15) {
        const uint32_t mms = (w.tim->CR2 & TIM_CR2_MMS_Msk) >> TIM_CR2_MMS_Pos;
        return mms == TIM_TRGO_OCREF(w.channel) ? static_cast<int>(SIM_ADC_EXTSEL_TIM15_TRGO) : -1;
    }
    if (w.lptim == LPTIM1 && w.channel == 0) return static_cast<int>(SIM_ADC_EXTSEL_LPTIM1_CH1);
    return -1;
}

}  // namespace sim::io
