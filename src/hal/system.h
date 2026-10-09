#ifndef BUCKY_SYSTEM_H
#define BUCKY_SYSTEM_H

#include <stm32g4xx.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SYS_CLOCK_HZ  170000000U
#define PCLK1_HZ      170000000U
#define PCLK2_HZ      170000000U

void system_init(void);

uint32_t millis(void);
uint32_t micros(void);
void delay(uint32_t ms);
void delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#ifndef constrain
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

#endif // BUCKY_SYSTEM_H
