#ifndef BUCKY_MCU_H
#define BUCKY_MCU_H

// Single place that knows which STM32 family we are building for. Drivers
// include this instead of a family header and branch on MCU_* feature macros.

#if defined(STM32G4xx)
  #include <stm32g4xx.h>
  #define MCU_FAMILY_G4    1
  #define MCU_IRQ_COUNT    102U   /* FMAC_IRQn + 1 */
#elif defined(STM32H5xx)
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

/* G4 routes DMA requests through DMAMUX; H5 has GPDMA with REQSEL per channel. */
#ifdef GPDMA1
  #define MCU_HAS_GPDMA  1
#else
  #define MCU_HAS_GPDMA  0
#endif

#endif // BUCKY_MCU_H
