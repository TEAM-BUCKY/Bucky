#ifndef BUCKY_LPTIM_H
#define BUCKY_LPTIM_H

#include <stdbool.h>
#include <PinNames.h>

#include <hardware/io/mcu.h>

#if defined(MCU_FAMILY_H5)

#ifdef __cplusplus
extern "C" {
#endif

void lptim_pin_connect(PinName pin, uint8_t af);

bool lptim_pwm_start(LPTIM_TypeDef* lptim, uint8_t channel, uint32_t freq_hz, uint16_t duty_permille);

void lptim_stop(LPTIM_TypeDef* lptim);

#ifdef __cplusplus
}
#endif

#endif // MCU_FAMILY_H5

#endif // BUCKY_LPTIM_H
