#ifndef BUCKY_IRQ_H
#define BUCKY_IRQ_H

#include <stdbool.h>
#include <PinNames.h>

#include "hardware/io/mcu.h"

// Runtime interrupt registration.
//
// The first attach copies the vector table into RAM and points VTOR at it, so
// any peripheral IRQ can be bound to a (handler, context) pair at runtime
// without defining a strongly-named *_IRQHandler symbol. Vectors that were
// never attached keep the core's original handler (SysTick, USB, ...).
//
// Pin interrupts go through exti_attach(), which owns the EXTI line config on
// both families (G4: SYSCFG + shared 9_5/15_10 vectors; H5: EXTI->EXTICR and
// one vector per line). Do not mix it with Arduino attachInterrupt() on the
// same EXTI line: whichever registered last owns the vector.

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*IrqHandler)(void* ctx);

typedef enum {
    EXTI_RISING  = 1,
    EXTI_FALLING = 2,
    EXTI_BOTH    = 3,
} ExtiEdge;

// Bind handler(ctx) to irqn, set its priority and enable it in the NVIC.
void irq_attach(IRQn_Type irqn, IrqHandler handler, void* ctx, uint8_t priority);

// Disable irqn in the NVIC and restore the vector it had before irq_attach().
void irq_detach(IRQn_Type irqn);

// Configure the pin's EXTI line and call handler(ctx) on the chosen edges.
// Returns false when the line (pin number 0-15) is already claimed by another
// port, e.g. PA2 and PB2 both live on EXTI2.
bool exti_attach(PinName pin, ExtiEdge edge, IrqHandler handler, void* ctx, uint8_t priority);

void exti_detach(PinName pin);

// True when exti_attach() would succeed for this pin.
bool exti_line_free(PinName pin);

#ifdef __cplusplus
}
#endif

#endif // BUCKY_IRQ_H
