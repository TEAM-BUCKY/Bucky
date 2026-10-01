#ifndef BUCKY_LPTIM_H
#define BUCKY_LPTIM_H

#include <stdbool.h>
#include <PinNames.h>

#include "hardware/io/mcu.h"

// PWM on the low-power timers of the STM32H5 (LPTIM1..LPTIM6).
//
// These are a separate IP from TIMx: 16-bit counter, power-of-two prescaler,
// and ARR/CCRx can only be written while the timer is enabled (each write is
// acknowledged by an xOK flag), while CFGR/CCMR1 can only be written while it
// is disabled. The driver hides that ordering.
//
// Waveform: output high while CNT < CCRx, low for the rest of the period.
// LPTIM1_CH1 / LPTIM2_CH1 can also trigger the ADC, see ADC_EXTERNALTRIG_LPTIMx_CH1.
//
// The core's pin map does not list LPTIM pins, so the alternate function
// number comes from the datasheet and is passed to lptim_pin_connect().

#if defined(MCU_FAMILY_H5)

#ifdef __cplusplus
extern "C" {
#endif

// Route a pin to the LPTIM alternate function.
void lptim_pin_connect(PinName pin, uint8_t af);

// Configure and start continuous PWM on channel 0 or 1. duty_permille is the
// high time (0..1000). Returns false when freq_hz cannot be produced with a
// 16-bit counter (too low even at /128, or too high for 2 ticks per period).
bool lptim_pwm_start(LPTIM_TypeDef* lptim, uint8_t channel, uint32_t freq_hz, uint16_t duty_permille);

void lptim_stop(LPTIM_TypeDef* lptim);

#ifdef __cplusplus
}
#endif

#endif // MCU_FAMILY_H5

#endif // BUCKY_LPTIM_H
