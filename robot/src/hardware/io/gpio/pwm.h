#ifndef BUCKY_PWM_H
#define BUCKY_PWM_H

#include <Arduino.h>
#include <PeripheralPins.h>
#include <pinmap.h>

#include "hardware/io/mcu.h"
#include "hardware/io/gpio/gpio.h"
#include "hardware/io/timer/Timer.h"

#include "optimizations/bitboard.h"
#include "optimizations/optimizations.h"

#if !defined(HRTIM1)
// Keeps PwmPin's layout identical on families without HRTIM (e.g. STM32H5).
typedef struct HrtimTimerUnavailable HRTIM_Timerx_TypeDef;
#endif

#define PWM_SYNC_MAX_PINS   8
#define PWM_SYNC_MAX_TIMERS 4
#define PWM_SYNC_NONE       0xFF

enum : uint8_t {
    PWM_BACKEND_NONE = 0,
    PWM_BACKEND_TIM = 1,
    PWM_BACKEND_HRTIM = 2,
};

typedef struct {
    TIM_TypeDef *timer;
    volatile uint32_t *ccr;
    uint8_t channel;
    uint8_t syncIndex;
    uint8_t complementary;
    PinName mappedPin;
    HRTIM_Timerx_TypeDef *hrtimTimer;
    uint16_t hrtimOutputEnableMask;
    uint32_t hrtimCounterEnableMask;
    uint8_t backend;
} PwmPin;

#if defined(HRTIM1)
// All motor PWM pins are routed through HRTIM1 so the three timers (A/C/F) share
// one clocking/DLL setup and can be synchronised. The core pin map does not
// list HRTIM, so every pin carries its own timer + bit masks and is routed to
// AF13 directly.
typedef struct {
    PinName pin;
    HRTIM_Timerx_TypeDef *timer;
    uint8_t channel;                    // 0 = CHx1, 1 = CHx2
    uint16_t outputEnableMask;          // OENR bit for this output
    uint32_t counterEnableMask;         // MCR bit that starts the owning timer
} HrtimMotorPinInfo;

static const HrtimMotorPinInfo hrtim_motor_pins[] = {
    {PA_8,  HRTIM1_TIMA, 0, HRTIM_OENR_TA1OEN, HRTIM_MCR_TACEN},
    {PA_9,  HRTIM1_TIMA, 1, HRTIM_OENR_TA2OEN, HRTIM_MCR_TACEN},
    {PB_12, HRTIM1_TIMC, 0, HRTIM_OENR_TC1OEN, HRTIM_MCR_TCCEN},
    {PB_13, HRTIM1_TIMC, 1, HRTIM_OENR_TC2OEN, HRTIM_MCR_TCCEN},
    {PC_6,  HRTIM1_TIMF, 0, HRTIM_OENR_TF1OEN, HRTIM_MCR_TFCEN},
    {PC_7,  HRTIM1_TIMF, 1, HRTIM_OENR_TF2OEN, HRTIM_MCR_TFCEN},
    {NC, nullptr, 0, 0, 0}
};

static FORCE_INLINE const HrtimMotorPinInfo *hrtim_motor_lookup(const PinName pn)
{
    for (const HrtimMotorPinInfo *e = hrtim_motor_pins; e->pin != NC; e++) {
        if (e->pin == pn) return e;
    }
    return nullptr;
}

// If CMP >= PER the reset event either never fires or ties with the period-set
// event (reset wins on HRTIM -> 0% duty). Push CMP above PER so the compare
// never triggers and the output stays HIGH.
static FORCE_INLINE uint32_t hrtim_clamp_compare(const PwmPin *pw, const uint32_t value)
{
    const uint32_t per = pw->hrtimTimer->PERxR;
    return value >= per ? per + 1U : value;
}

// CMP=0 is below the HRTIM minimum (3 ticks), so the compare event never fires
// and the output would stay HIGH: disable the output instead to force it LOW.
static FORCE_INLINE void hrtim_write_output(volatile uint32_t *ccr, const uint16_t outputMask, const uint32_t value)
{
    if (value == 0U) {
        setMask(HRTIM1->sCommonRegs.ODISR, outputMask);
    } else {
        *ccr = value;
        setMask(HRTIM1->sCommonRegs.OENR, outputMask);
    }
}
#endif

typedef struct {
    volatile uint32_t *ccr;
    uint32_t value;
    uint16_t hrtimOutputEnableMask;
} PwmStagedPin;

typedef struct {
    PwmStagedPin pins[PWM_SYNC_MAX_PINS];
    uint8_t pinCount;
    uint8_t dirtyPins;
    TIM_TypeDef *timers[PWM_SYNC_MAX_TIMERS];
    uint8_t timerCount;
} PwmSyncState;

inline PwmSyncState pwm_sync = {};

static FORCE_INLINE bool pwm_same_gpio_pin(const PinName a, const PinName b)
{
    return STM_PORT(a) == STM_PORT(b) && STM_PIN(a) == STM_PIN(b);
}

static FORCE_INLINE PwmPin pwm_pin_none()
{
    PwmPin pw = {};
    pw.mappedPin = NC;
    pw.syncIndex = PWM_SYNC_NONE;
    return pw;
}

static FORCE_INLINE PwmPin pwm_pin_from_map(const PinName mappedPin, TIM_TypeDef *timer,
                                            const uint8_t channel, const bool complementary)
{
    PwmPin pw = pwm_pin_none();
    pw.timer = timer;
    pw.channel = channel;
    pw.complementary = complementary;
    pw.mappedPin = mappedPin;
    pw.ccr = tim_ccr(timer, channel);
    pw.backend = PWM_BACKEND_TIM;
    return pw;
}

// Exact lookup: `pn` may carry an _ALTn suffix to choose the timer, e.g.
// PB_14_ALT2 selects TIM12_CH1 instead of the default TIM1_CH2N.
static FORCE_INLINE PwmPin pwm_pin_init(const PinName pn)
{
#if defined(HRTIM1)
    if (const HrtimMotorPinInfo *info = hrtim_motor_lookup(pn); info != nullptr) {
        PwmPin pw = pwm_pin_none();
        pw.hrtimTimer = info->timer;
        pw.channel = info->channel;
        pw.mappedPin = pn;
        pw.ccr = (info->channel == 0U) ? &info->timer->CMP1xR : &info->timer->CMP2xR;
        pw.hrtimOutputEnableMask = info->outputEnableMask;
        pw.hrtimCounterEnableMask = info->counterEnableMask;
        pw.backend = PWM_BACKEND_HRTIM;
        return pw;
    }
#endif

    uint8_t channel = 0;
    bool complementary = false;
    TIM_TypeDef *timer = tim_from_pin(pn, &channel, &complementary);
    if (timer == nullptr) {
        return pwm_pin_none();
    }
    return pwm_pin_from_map(pn, timer, channel, complementary);
}

// Arduino pin number: prefer an advanced timer among every _ALTn entry of the pad.
static FORCE_INLINE PwmPin pwm_pin_init(const int pin)
{
    const PinName pn = digitalPinToPinName(pin);

#if defined(HRTIM1)
    if (hrtim_motor_lookup(pn) != nullptr) {
        return pwm_pin_init(pn);
    }
#endif

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

#if defined(HRTIM1)
static FORCE_INLINE void pwm_init_hrtim(const PwmPin *pw, const uint32_t resolution)
{
    __HAL_RCC_HRTIM1_CLK_ENABLE();

    // HRTIM cannot drive its outputs until the DLL is calibrated. Do the
    // one-shot calibration + enable periodic recalibration the first time
    // any HRTIM pin is initialised.
    if ((HRTIM1->sCommonRegs.DLLCR & HRTIM_DLLCR_CALEN) == 0U) {
        HRTIM1->sCommonRegs.DLLCR = HRTIM_DLLCR_CALEN | HRTIM_DLLCR_CAL;
        while ((HRTIM1->sCommonRegs.ISR & HRTIM_ISR_DLLRDY) == 0U) {}
        HRTIM1->sCommonRegs.ICR = HRTIM_ICR_DLLRDYC;
    }

    pw->hrtimTimer->TIMxCR = (pw->hrtimTimer->TIMxCR & ~HRTIM_TIMCR_CK_PSC) | HRTIM_TIMCR_CONT;
    pw->hrtimTimer->PERxR = resolution;

    if (pw->channel == 0U) {
        pw->hrtimTimer->SETx1R = HRTIM_SET1R_PER;
        pw->hrtimTimer->RSTx1R = HRTIM_RST1R_CMP1;
    } else {
        pw->hrtimTimer->SETx2R = HRTIM_SET2R_PER;
        pw->hrtimTimer->RSTx2R = HRTIM_RST2R_CMP2;
    }

    *pw->ccr = 0;
    setMask(HRTIM1->sMasterRegs.MCR, pw->hrtimCounterEnableMask);

    pin_function(pw->mappedPin, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF13_HRTIM1));
}
#endif

// Start PWM with a period of (resolution + 1) counts. TIMx runs those at
// freq * (resolution + 1) Hz as far as the prescaler allows; HRTIM ignores
// freq and counts at its own clock.
static FORCE_INLINE void pwm_init(const PwmPin *pw, const uint32_t freq, const uint32_t resolution)
{
    if (pw->ccr == nullptr) {
        return;
    }

    gpio_hold_low(pw->mappedPin);   // until the timer takes the pad over

#if defined(HRTIM1)
    if (pw->backend == PWM_BACKEND_HRTIM) {
        pwm_init_hrtim(pw, resolution);
        return;
    }
#endif

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

static FORCE_INLINE void pwm_write(const PwmPin *pw, const uint32_t value)
{
    if (pw->ccr == nullptr) {
        return;
    }
#if defined(HRTIM1)
    if (pw->backend == PWM_BACKEND_HRTIM) {
        hrtim_write_output(pw->ccr, pw->hrtimOutputEnableMask, hrtim_clamp_compare(pw, value));
        return;
    }
#endif
    *pw->ccr = value;
}

static FORCE_INLINE void pwm_sync_register(PwmPin *pw)
{
    if (pw->ccr == nullptr) {
        return;
    }

    if (pwm_sync.pinCount < PWM_SYNC_MAX_PINS) {
        pw->syncIndex = pwm_sync.pinCount;
        pwm_sync.pins[pwm_sync.pinCount] = {pw->ccr, 0, pw->hrtimOutputEnableMask};
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

static FORCE_INLINE void pwm_stage(const PwmPin *pw, const uint32_t value)
{
    if (pw->syncIndex == PWM_SYNC_NONE) {
        return;
    }
    uint32_t staged = value;
#if defined(HRTIM1)
    if (pw->backend == PWM_BACKEND_HRTIM) {
        staged = hrtim_clamp_compare(pw, value);
    }
#endif
    pwm_sync.pins[pw->syncIndex].value = staged;
    setBit(pwm_sync.dirtyPins, pw->syncIndex);
}

static FORCE_INLINE void pwm_commit()
{
    __disable_irq();

    uint8_t dirty = pwm_sync.dirtyPins;
    Bitloop(dirty) {
        const PwmStagedPin &pin = pwm_sync.pins[getLSB(dirty)];
#if defined(HRTIM1)
        if (pin.hrtimOutputEnableMask != 0) {
            hrtim_write_output(pin.ccr, pin.hrtimOutputEnableMask, pin.value);
            continue;
        }
#endif
        *pin.ccr = pin.value;
    }
    pwm_sync.dirtyPins = 0;

    __enable_irq();
}

#endif // BUCKY_PWM_H
