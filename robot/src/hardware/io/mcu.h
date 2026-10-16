#ifndef BUCKY_MCU_H
#define BUCKY_MCU_H

// Single place that knows which STM32 family we are building for. Drivers
// include this instead of a family header and branch on MCU_* feature macros.

#if defined(STM32H5xx)
  #include <stm32h5xx.h>
  #define MCU_FAMILY_H5    1
  #define MCU_IRQ_COUNT    131U   /* LPTIM6_IRQn + 1 */
#else
  #error "Unsupported MCU family: add it to hardware/io/mcu.h"
#endif

#include <stm32yyxx_ll_rcc.h>

/* Hardware accelerators that only some families have. */
#ifdef CORDIC
  #define MCU_HAS_CORDIC 1
#else
  #define MCU_HAS_CORDIC 0
#endif

#endif // BUCKY_MCU_H
