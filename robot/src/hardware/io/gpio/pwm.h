#ifndef BUCKY_PWM_H
#define BUCKY_PWM_H

#include <Arduino.h>
#include <PeripheralPins.h>

#include "hardware/io/mcu.h"
#include "hardware/io/timer/Timer.h"

#include "optimizations/bitboard.h"
#include "optimizations/optimizations.h"

#if !defined(HRTIM1)
// Keeps PwmPin's layout identical on families without HRTIM (e.g. STM32H5).
typedef struct HrtimTimerUnavailable HRTIM_Timerx_TypeDef;
#endif

#define PWM_SYNC_MAX_PINS   8
#define PWM_SYNC_MAX_TIMERS 4

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

enum : uint8_t {
    PWM_BACKEND_NONE = 0,
    PWM_BACKEND_TIM = 1,
    PWM_BACKEND_HRTIM = 2,
};

#if defined(HRTIM1)
// All motor PWM pins are routed through HRTIM1 so the three timers (A/C/F) share
// one clocking/DLL setup and can be synchronised. Keep the pinmap flat — one
// entry per pin — and carry the HRTIM timer + bit masks alongside so pwm_pin_init
// is a straight table lookup instead of a chain of if-branches.
typedef struct {
    PinName pin;
    HRTIM_Timerx_TypeDef *timer;
    uint8_t channel;                    // 0 = CHx1, 1 = CHx2
    uint16_t outputEnableMask;          // OENR bit for this output
    uint32_t counterEnableMask;         // MCR bit that starts the owning timer
} HrtimMotorPinInfo;

static const PinMap PinMap_HRTIM_MOTOR[] = {
    {PA_8,  (void *)HRTIM1_BASE, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF13_HRTIM1, 1, 0)}, // HRTIM1_CHA1
    {PA_9,  (void *)HRTIM1_BASE, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF13_HRTIM1, 2, 0)}, // HRTIM1_CHA2
    {PB_12, (void *)HRTIM1_BASE, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF13_HRTIM1, 1, 0)}, // HRTIM1_CHC1
    {PB_13, (void *)HRTIM1_BASE, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF13_HRTIM1, 2, 0)}, // HRTIM1_CHC2
    {PC_6,  (void *)HRTIM1_BASE, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF13_HRTIM1, 1, 0)}, // HRTIM1_CHF1
    {PC_7,  (void *)HRTIM1_BASE, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF13_HRTIM1, 2, 0)}, // HRTIM1_CHF2
    {NC, NP, 0}
};

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

inline PwmSyncState pwm_sync = {nullptr};

static FORCE_INLINE void timer_enable(TIM_TypeDef *tim)
{
    setMask(tim->CR1, TIM_CR1_CEN);
}

static FORCE_INLINE void timer_disable(TIM_TypeDef *tim)
{
    clearMask(tim->CR1, TIM_CR1_CEN);
}

static FORCE_INLINE uint8_t pwm_timer_rank(const TIM_TypeDef *tim)
{
    return IS_TIM_BREAK_INSTANCE(tim) ? 3 : 1;
}

static FORCE_INLINE bool pwm_same_gpio_pin(const PinName a, const PinName b)
{
    return STM_PORT(a) == STM_PORT(b) && STM_PIN(a) == STM_PIN(b);
}

static FORCE_INLINE PwmPin pwm_pin_none()
{
    PwmPin pw;
    pw.timer = nullptr;
    pw.ccr = nullptr;
    pw.channel = 0;
    pw.complementary = 0;
    pw.mappedPin = NC;
    pw.hrtimTimer = nullptr;
    pw.hrtimOutputEnableMask = 0;
    pw.hrtimCounterEnableMask = 0;
    pw.backend = PWM_BACKEND_NONE;
    pw.syncIndex = 0xFF;
    return pw;
}

static FORCE_INLINE PwmPin pwm_pin_from_map(const PinName mappedPin, const TIM_TypeDef *timer,
                                            const uint8_t channel, const bool complementary)
{
    PwmPin pw = pwm_pin_none();
    pw.timer = const_cast<TIM_TypeDef *>(timer);
    pw.channel = channel;
    pw.complementary = complementary;
    pw.mappedPin = mappedPin;
    pw.ccr = &pw.timer->CCR1 + pw.channel;
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
    const TIM_TypeDef *timer = tim_from_pin(pn, &channel, &complementary);
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
    uint8_t bestRank = 0;

    for (const PinMap *map = PinMap_TIM; map->pin != NC; map++) {
        if (!pwm_same_gpio_pin(map->pin, pn)) {
            continue;
        }

        const auto *candidateTimer = reinterpret_cast<TIM_TypeDef *>(map->peripheral);
        const uint8_t rank = pwm_timer_rank(candidateTimer);
        if (bestEntry == nullptr || rank > bestRank || (rank == bestRank && map->pin == pn)) {
            bestEntry = map;
            bestRank = rank;
        }
    }

    if (bestEntry == nullptr) {
        return pwm_pin_none();
    }
    return pwm_pin_from_map(bestEntry->pin, reinterpret_cast<TIM_TypeDef *>(bestEntry->peripheral),
                            STM_PIN_CHANNEL(bestEntry->function) - 1,
                            STM_PIN_INVERTED(bestEntry->function));
}

static FORCE_INLINE void pwm_init(const PwmPin *pw, const uint32_t freq, const uint32_t resolution)
{
    if (pw->ccr == nullptr) {
        return;
    }

    // Hold the pad low until the timer takes it over.
    const PinName pad = static_cast<PinName>(pw->mappedPin & PNAME_MASK);
    pinMode(pinNametoDigitalPin(pad), OUTPUT);
    digitalWriteFast(pad, LOW);

#if defined(HRTIM1)
    if (pw->backend == PWM_BACKEND_HRTIM && pw->hrtimTimer != nullptr) {
        (void)freq;
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

        pinmap_pinout(pw->mappedPin, PinMap_HRTIM_MOTOR);
        return;
    }
#endif

    if (pw->timer == nullptr) {
        return;
    }

    tim_clock_enable(pw->timer);

    const uint32_t timerClk = tim_clock_hz(pw->timer);
    const uint32_t periodTicks = freq * (resolution + 1);
    if (timerClk < periodTicks || periodTicks == 0U) {
        pw->timer->PSC = 0;
    } else {
        pw->timer->PSC = timerClk / periodTicks - 1U;
    }
    pw->timer->ARR = resolution;

    const uint8_t ch = pw->channel;

    volatile uint32_t *ccmr = &pw->timer->CCMR1 + (ch >> 1);
    const uint8_t shift = (ch & 1) * 8;
    writeField(*ccmr, 0xFFU, shift, 0x68U);

    *pw->ccr = 0;

    setBit(pw->timer->CCER, ch * 4 + pw->complementary * 2);
    if (pw->complementary) {
        setBit(pw->timer->CCER, ch * 4 + 3);  // CCxNP: invert complementary polarity
    }

    // TIM1/8/15/16/17/20: outputs stay off until the main output enable is set.
    if (IS_TIM_BREAK_INSTANCE(pw->timer)) {
        setMask(pw->timer->BDTR, TIM_BDTR_MOE);
    }

    timer_enable(pw->timer);

    pinmap_pinout(pw->mappedPin, PinMap_TIM);
}

static FORCE_INLINE void pwm_write(const PwmPin *pw, const uint32_t value)
{
    if (pw->ccr == nullptr) {
        return;
    }
#if defined(HRTIM1)
    if (pw->backend == PWM_BACKEND_HRTIM) {
        if (value == 0) {
            // CMP=0 is below the HRTIM minimum (3 ticks) so the compare event
            // never fires and the output stays HIGH.  Disable the output instead
            // to force the pin to its inactive (LOW) level.
            setMask(HRTIM1->sCommonRegs.ODISR, pw->hrtimOutputEnableMask);
        } else {
            // If CMP >= PER the reset event either never fires or ties with the
            // period-set event (reset wins on HRTIM → 0% duty). Push CMP above
            // PER so the compare never triggers → output stays HIGH.
            const uint32_t per = pw->hrtimTimer->PERxR;
            *pw->ccr = (value >= per) ? (per + 1U) : value;
            setMask(HRTIM1->sCommonRegs.OENR, pw->hrtimOutputEnableMask);
        }
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
        pwm_sync.pins[pwm_sync.pinCount].ccr = pw->ccr;
        pwm_sync.pins[pwm_sync.pinCount].value = 0;
        pwm_sync.pins[pwm_sync.pinCount].hrtimOutputEnableMask = pw->hrtimOutputEnableMask;
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
        pwm_sync.timers[pwm_sync.timerCount] = pw->timer;
        pwm_sync.timerCount++;
    }
}

static FORCE_INLINE void pwm_sync_timers()
{
    __disable_irq();

    for (uint8_t i = 0; i < pwm_sync.timerCount; i++) {
        timer_disable(pwm_sync.timers[i]);
    }

    for (uint8_t i = 0; i < pwm_sync.timerCount; i++) {
        pwm_sync.timers[i]->CNT = 0;
        timer_enable(pwm_sync.timers[i]);
    }

    __enable_irq();
}

static FORCE_INLINE void pwm_stage(const PwmPin *pw, const uint32_t value)
{
    if (pw->syncIndex == 0xFF) {
        return;
    }
    uint32_t staged = value;
#if defined(HRTIM1)
    if (pw->backend == PWM_BACKEND_HRTIM && value != 0U) {
        // Match pwm_write(): CMP == PER makes the reset tie the period-set and
        // collapses duty to 0%. Push just past PER so the compare never fires.
        const uint32_t per = pw->hrtimTimer->PERxR;
        if (staged >= per) staged = per + 1U;
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
        uint8_t i = GetLSB(dirty);
#if defined(HRTIM1)
        if (pwm_sync.pins[i].hrtimOutputEnableMask != 0) {
            if (pwm_sync.pins[i].value == 0) {
                setMask(HRTIM1->sCommonRegs.ODISR, pwm_sync.pins[i].hrtimOutputEnableMask);
            } else {
                *pwm_sync.pins[i].ccr = pwm_sync.pins[i].value;
                setMask(HRTIM1->sCommonRegs.OENR, pwm_sync.pins[i].hrtimOutputEnableMask);
            }
        } else {
            *pwm_sync.pins[i].ccr = pwm_sync.pins[i].value;
        }
#else
        *pwm_sync.pins[i].ccr = pwm_sync.pins[i].value;
#endif
    }
    pwm_sync.dirtyPins = 0;

    __enable_irq();
}

#endif // BUCKY_PWM_H
