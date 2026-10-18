#ifndef BUCKY_PWM_H
#define BUCKY_PWM_H

#include <Arduino.h>
#include <PeripheralPins.h>
#include <pinmap.h>

#include <hardware/io/mcu.h>
#include <hardware/io/gpio/gpio.h>
#include <hardware/io/timer/Timer.h>

#include <optimizations/bitboard.h>
#include <optimizations/optimizations.h>

#define PWM_SYNC_MAX_PINS   8
#define PWM_SYNC_MAX_TIMERS 4
#define PWM_SYNC_NONE       0xFF

typedef struct {
    TIM_TypeDef *timer;
    volatile uint32_t *ccr;
    uint8_t channel;
    uint8_t syncIndex;
    uint8_t complementary;
    PinName mappedPin;
} PWMPin;

typedef struct {
    volatile uint32_t *ccr;
    uint32_t value;
} PWMStagedPin;

typedef struct {
    PWMStagedPin pins[PWM_SYNC_MAX_PINS];
    uint8_t pinCount;
    uint8_t dirtyPins;
    TIM_TypeDef *timers[PWM_SYNC_MAX_TIMERS];
    uint8_t timerCount;
} PWMSyncState;

inline PWMSyncState pwm_sync = {};

static FORCE_INLINE bool pwm_same_gpio_pin(const PinName a, const PinName b)
{
    return STM_PORT(a) == STM_PORT(b) && STM_PIN(a) == STM_PIN(b);
}

static FORCE_INLINE PWMPin pwm_pin_none()
{
    PWMPin pw = {};
    pw.mappedPin = NC;
    pw.syncIndex = PWM_SYNC_NONE;
    return pw;
}

static FORCE_INLINE PWMPin pwm_pin_from_map(const PinName mappedPin, TIM_TypeDef *timer,
                                            const uint8_t channel, const bool complementary)
{
    PWMPin pw = pwm_pin_none();
    pw.timer = timer;
    pw.channel = channel;
    pw.complementary = complementary;
    pw.mappedPin = mappedPin;
    pw.ccr = tim_ccr(timer, channel);
    return pw;
}

static FORCE_INLINE PWMPin pwm_pin_init(const PinName pn)
{
    uint8_t channel = 0;
    bool complementary = false;
    TIM_TypeDef *timer = tim_from_pin(pn, &channel, &complementary);
    if (timer == nullptr) {
        return pwm_pin_none();
    }
    return pwm_pin_from_map(pn, timer, channel, complementary);
}

static FORCE_INLINE PWMPin pwm_pin_init(const int pin)
{
    const PinName pn = digitalPinToPinName(pin);

    const PinMap *bestEntry = nullptr;
    bool bestAdvanced = false;

    for (const PinMap *map = PinMap_TIM; map->pin != NC; map++) {
        if (!pwm_same_gpio_pin(map->pin, pn)) {
            continue;
        }

        const bool advanced = IS_TIM_BREAK_INSTANCE(static_cast<TIM_TypeDef *>(map->peripheral));
        if (bestEntry == nullptr || (advanced && !bestAdvanced) || (advanced == bestAdvanced && map->pin == pn)) {
            bestEntry = map;
            bestAdvanced = advanced;
        }
    }

    if (bestEntry == nullptr) {
        return pwm_pin_none();
    }
    return pwm_pin_from_map(bestEntry->pin, static_cast<TIM_TypeDef *>(bestEntry->peripheral),
                            STM_PIN_CHANNEL(bestEntry->function) - 1,
                            STM_PIN_INVERTED(bestEntry->function));
}

static FORCE_INLINE void pwm_init(const PWMPin *pw, const uint32_t freq, const uint32_t resolution)
{
    if (pw->ccr == nullptr) {
        return;
    }

    gpio_hold_low(pw->mappedPin);   // until the timer takes the pad over

    TIM_TypeDef *tim = pw->timer;
    tim_clock_enable(tim);

    const uint32_t timerClk = tim_clock_hz(tim);
    const uint32_t countHz = freq * (resolution + 1);
    tim->PSC = (countHz == 0U || timerClk < countHz) ? 0U : timerClk / countHz - 1U;
    tim->ARR = resolution;

    *pw->ccr = 0;
    tim_pwm_channel_enable(tim, pw->channel, pw->complementary);
    tim_start(tim);

    tim_pin_connect(pw->mappedPin);
}

static FORCE_INLINE void pwm_write(const PWMPin *pw, const uint32_t value)
{
    if (pw->ccr == nullptr) {
        return;
    }
    *pw->ccr = value;
}

static FORCE_INLINE void pwm_sync_register(PWMPin *pw)
{
    if (pw->ccr == nullptr) {
        return;
    }

    if (pwm_sync.pinCount < PWM_SYNC_MAX_PINS) {
        pw->syncIndex = pwm_sync.pinCount;
        pwm_sync.pins[pwm_sync.pinCount] = {pw->ccr, 0};
        pwm_sync.pinCount++;
    }

    if (pw->timer == nullptr) {
        return;
    }

    for (uint8_t i = 0; i < pwm_sync.timerCount; i++) {
        if (pwm_sync.timers[i] == pw->timer) {
            return;
        }
    }

    if (pwm_sync.timerCount < PWM_SYNC_MAX_TIMERS) {
        pwm_sync.timers[pwm_sync.timerCount++] = pw->timer;
    }
}

static FORCE_INLINE void pwm_sync_timers()
{
    __disable_irq();

    for (uint8_t i = 0; i < pwm_sync.timerCount; i++) {
        tim_stop(pwm_sync.timers[i]);
    }

    for (uint8_t i = 0; i < pwm_sync.timerCount; i++) {
        pwm_sync.timers[i]->CNT = 0;
        tim_start(pwm_sync.timers[i]);
    }

    __enable_irq();
}

static FORCE_INLINE void pwm_stage(const PWMPin *pw, const uint32_t value)
{
    if (pw->syncIndex == PWM_SYNC_NONE) {
        return;
    }
    pwm_sync.pins[pw->syncIndex].value = value;
    setBit(pwm_sync.dirtyPins, pw->syncIndex);
}

static FORCE_INLINE void pwm_commit()
{
    __disable_irq();

    uint8_t dirty = pwm_sync.dirtyPins;
    Bitloop(dirty) {
        const auto & [ccr, value] = pwm_sync.pins[getLSB(dirty)];
        *ccr = value;
    }
    pwm_sync.dirtyPins = 0;

    __enable_irq();
}

#endif // BUCKY_PWM_H
