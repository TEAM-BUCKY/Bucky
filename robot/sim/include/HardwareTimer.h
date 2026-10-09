/* Host stand-in for stm32duino's HardwareTimer, reduced to what Sonar.cpp uses (input capture
 * on both edges, for an echo on PA_10). On a capture edge the simulator latches the 1 MHz
 * counter (micros() & 0xFFFF, matching the prescaler Sonar.cpp programs) into CCRn and calls the
 * attached callback, like the real capture interrupt. */
#ifndef BUCKY_SIM_HARDWARETIMER_H
#define BUCKY_SIM_HARDWARETIMER_H

#include <Arduino.h>

typedef enum {
    TIMER_DISABLED,
    TIMER_OUTPUT_COMPARE,
    TIMER_INPUT_CAPTURE_RISING,
    TIMER_INPUT_CAPTURE_FALLING,
    TIMER_INPUT_CAPTURE_BOTHEDGE,
} TimerModes_t;

typedef enum { TICK_FORMAT, MICROSEC_FORMAT, HERTZ_FORMAT } TimerFormat_t;

typedef void (*callback_function_t)(void);

class HardwareTimer {
public:
    explicit HardwareTimer(TIM_TypeDef* instance);
    ~HardwareTimer();

    void setMode(uint32_t channel, TimerModes_t mode, PinName pin = NC);
    uint32_t getTimerClkFreq();
    void setPrescaleFactor(uint32_t prescaler);
    void setOverflow(uint32_t val, TimerFormat_t format = TICK_FORMAT);
    void attachInterrupt(uint32_t channel, callback_function_t callback);
    void resume();
    void pause();

    TIM_TypeDef* instance() const { return tim; }

private:
    TIM_TypeDef* tim;
};

#endif
