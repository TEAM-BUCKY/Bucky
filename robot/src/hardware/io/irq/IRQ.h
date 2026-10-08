#ifndef BUCKY_IRQ_H
#define BUCKY_IRQ_H

#include <stdbool.h>
#include <PinNames.h>

#include "hardware/io/mcu.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline uint32_t irq_lock(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void irq_restore(const uint32_t primask)
{
    __set_PRIMASK(primask);
}

typedef void (*IrqHandler)(void* ctx);

typedef enum {
    EXTI_RISING  = 1,
    EXTI_FALLING = 2,
    EXTI_BOTH    = 3,
} ExtiEdge;

void irq_attach(IRQn_Type irqn, IrqHandler handler, void* ctx, uint8_t priority);

void irq_detach(IRQn_Type irqn);

bool exti_attach(PinName pin, ExtiEdge edge, IrqHandler handler, void* ctx, uint8_t priority);

void exti_detach(PinName pin);

bool exti_line_free(PinName pin);

#ifdef __cplusplus
}
#endif

#endif // BUCKY_IRQ_H
