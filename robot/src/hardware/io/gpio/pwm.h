#ifndef BUCKY_PWM_H
#define BUCKY_PWM_H

#include <Arduino.h>
#include <stm32g4xx.h>
#include <PeripheralPins.h>

#include "optimizations/bitboard.h"
#include "optimizations/optimizations.h"

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
    if (tim == TIM1
#ifdef TIM8
        || tim == TIM8
#endif
#ifdef TIM20
        || tim == TIM20
#endif
    ) {
        return 3;
    }
    return 1;
}

static FORCE_INLINE bool pwm_same_gpio_pin(const PinName a, const PinName b)
{
    return STM_PORT(a) == STM_PORT(b) && STM_PIN(a) == STM_PIN(b);
}

static FORCE_INLINE PwmPin pwm_pin_init(const int pin)
{
    const PinName pn = digitalPinToPinName(pin);

#if defined(HRTIM1)
    if (const HrtimMotorPinInfo *info = hrtim_motor_lookup(pn); info != nullptr) {
        PwmPin pw;
        pw.timer = nullptr;
        pw.hrtimTimer = info->timer;
        pw.channel = info->channel;
        pw.syncIndex = 0xFF;
        pw.complementary = 0;
        pw.mappedPin = pn;
        pw.ccr = (info->channel == 0U) ? &info->timer->CMP1xR : &info->timer->CMP2xR;
        pw.hrtimOutputEnableMask = info->outputEnableMask;
        pw.hrtimCounterEnableMask = info->counterEnableMask;
        pw.backend = PWM_BACKEND_HRTIM;
        return pw;
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

    const uint32_t func = (bestEntry != nullptr) ? bestEntry->function : pinmap_function(pn, PinMap_TIM);
    PwmPin pw;
    pw.timer = (bestEntry != nullptr)
                   ? reinterpret_cast<TIM_TypeDef *>(bestEntry->peripheral)
                   : (TIM_TypeDef *)pinmap_peripheral(pn, PinMap_TIM);
    if (func == static_cast<uint32_t>(NC) || pw.timer == reinterpret_cast<TIM_TypeDef*>(NC)) {
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
    pw.channel = STM_PIN_CHANNEL(func) - 1;
    pw.complementary = STM_PIN_INVERTED(func);
    pw.mappedPin = (bestEntry != nullptr) ? bestEntry->pin : pn;
    pw.ccr = &pw.timer->CCR1 + pw.channel;
    pw.hrtimTimer = nullptr;
    pw.hrtimOutputEnableMask = 0;
    pw.hrtimCounterEnableMask = 0;
    pw.backend = PWM_BACKEND_TIM;
    pw.syncIndex = 0xFF;
    return pw;
}

static FORCE_INLINE void pwm_enable_clock(const TIM_TypeDef *tim)
{
    switch (reinterpret_cast<uintptr_t>(tim)) {
        case TIM2_BASE:  setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM2EN);  break;
        case TIM3_BASE:  setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM3EN);  break;
        case TIM4_BASE:  setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM4EN);  break;
#ifdef TIM5_BASE
        case TIM5_BASE:  setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM5EN);  break;
#endif
#ifdef TIM6_BASE
        case TIM6_BASE:  setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM6EN);  break;
#endif
#ifdef TIM7_BASE
        case TIM7_BASE:  setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM7EN);  break;
#endif
        case TIM1_BASE:  setMask(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);    break;
#ifdef TIM8_BASE
        case TIM8_BASE:  setMask(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);    break;
#endif
#ifdef TIM15_BASE
        case TIM15_BASE: setMask(RCC->APB2ENR, RCC_APB2ENR_TIM15EN);   break;
#endif
#ifdef TIM16_BASE
        case TIM16_BASE: setMask(RCC->APB2ENR, RCC_APB2ENR_TIM16EN);   break;
#endif
#ifdef TIM17_BASE
        case TIM17_BASE: setMask(RCC->APB2ENR, RCC_APB2ENR_TIM17EN);   break;
#endif
#ifdef TIM20_BASE
        case TIM20_BASE: setMask(RCC->APB2ENR, RCC_APB2ENR_TIM20EN);   break;
#endif
        default: break;
    }
}

static FORCE_INLINE uint32_t pwm_get_timer_clock(const TIM_TypeDef *tim)
{
    bool isApb2 = false;
    switch (reinterpret_cast<uintptr_t>(tim)) {
        case TIM1_BASE:
#ifdef TIM8_BASE
        case TIM8_BASE:
#endif
#ifdef TIM15_BASE
        case TIM15_BASE:
#endif
#ifdef TIM16_BASE
        case TIM16_BASE:
#endif
#ifdef TIM17_BASE
        case TIM17_BASE:
#endif
#ifdef TIM20_BASE
        case TIM20_BASE:
#endif
            isApb2 = true;
            break;
        default:
            isApb2 = false;
            break;
    }

    if (isApb2) {
        const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
        const uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos;
        return (ppre2 >= 4U) ? (pclk2 * 2U) : pclk2;
    }

    const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    const uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos;
    return (ppre1 >= 4U) ? (pclk1 * 2U) : pclk1;
}

static FORCE_INLINE void pwm_init(const PwmPin *pw, const int pin, const uint32_t freq, const uint32_t resolution)
{
    if (pw->ccr == nullptr) {
        return;
    }

    const PinName pn = digitalPinToPinName(pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

#if defined(HRTIM1)
    if (pw->backend == PWM_BACKEND_HRTIM && pw->hrtimTimer != nullptr) {
        (void)freq;
        setMask(RCC->APB2ENR, RCC_APB2ENR_HRTIM1EN);

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

    pwm_enable_clock(pw->timer);

    const uint32_t timerClk = pwm_get_timer_clock(pw->timer);
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

    if (pw->timer == TIM1
#ifdef TIM8
        || pw->timer == TIM8
#endif
#ifdef TIM20
        || pw->timer == TIM20
#endif
    ) {
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
