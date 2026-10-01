#include "IRQ.h"

#include <string.h>
#include <PortNames.h>

#include "optimizations/bitboard.h"
#include "optimizations/optimizations.h"

#define VECTOR_COUNT (16U + MCU_IRQ_COUNT)

typedef void (*Vector)(void);

typedef struct {
    IrqHandler handler;
    void*      ctx;
    Vector     original;
} IrqSlot;

/* VTOR requires the table on a 128/512-byte boundary depending on the core. */
static Vector ram_vectors[VECTOR_COUNT] __attribute__((aligned(512)));
static IrqSlot slots[MCU_IRQ_COUNT];
static bool relocated = false;

static void irq_trampoline(void)
{
    const IrqSlot* s = &slots[__get_IPSR() - 16U];
    s->handler(s->ctx);
}

static void relocate_vectors(void)
{
    if (relocated) return;

    const uint32_t primask = irq_lock();
    memcpy(ram_vectors, (const Vector*)SCB->VTOR, sizeof(ram_vectors));
    SCB->VTOR = (uint32_t)ram_vectors;
    __DSB();
    relocated = true;
    irq_restore(primask);
}

void irq_attach(const IRQn_Type irqn, const IrqHandler handler, void* ctx, const uint8_t priority)
{
    relocate_vectors();

    const uint32_t n = (uint32_t)irqn;
    NVIC_DisableIRQ(irqn);

    IrqSlot* s = &slots[n];
    if (ram_vectors[16U + n] != irq_trampoline)
        s->original = ram_vectors[16U + n];
    s->handler = handler;
    s->ctx     = ctx;
    ram_vectors[16U + n] = irq_trampoline;

    NVIC_SetPriority(irqn, priority);
    NVIC_EnableIRQ(irqn);
}

void irq_detach(const IRQn_Type irqn)
{
    if (!relocated) return;

    const uint32_t n = (uint32_t)irqn;
    NVIC_DisableIRQ(irqn);
    if (ram_vectors[16U + n] == irq_trampoline && slots[n].original != NULL)
        ram_vectors[16U + n] = slots[n].original;
    slots[n].handler = NULL;
    slots[n].ctx     = NULL;
}

/* ---------------------------------------------------------------- EXTI --- */

typedef struct {
    IrqHandler handler;
    void*      ctx;
    uint8_t    port;
} ExtiSlot;

static ExtiSlot exti_slots[16];
static uint16_t exti_used = 0;

#if defined(MCU_FAMILY_G4)

static IRQn_Type exti_irqn(const uint8_t line)
{
    if (line <= 4)  return (IRQn_Type)(EXTI0_IRQn + line);
    if (line <= 9)  return EXTI9_5_IRQn;
    return EXTI15_10_IRQn;
}

/* One dispatcher per vector; ctx carries the mask of lines that vector owns. */
static void exti_dispatch(void* ctx)
{
    uint32_t pending = EXTI->PR1 & EXTI->IMR1 & (uint32_t)(uintptr_t)ctx;
    EXTI->PR1 = pending;
    Bitloop(pending) {
        const ExtiSlot* s = &exti_slots[getLSB(pending)];
        if (s->handler) s->handler(s->ctx);
    }
}

static uint32_t exti_dispatch_ctx(const uint8_t line)
{
    if (line <= 4) return 1UL << line;
    if (line <= 9) return 0x03E0UL;
    return 0xFC00UL;
}

static void exti_select_port(const uint8_t line, const uint8_t port)
{
    setMask(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
    __DSB();
    writeField(SYSCFG->EXTICR[line >> 2], 0xFU, (line & 3U) * 4U, port);
}

static void exti_clear_pending(const uint32_t bit)
{
    EXTI->PR1 = bit;
}

#elif defined(MCU_FAMILY_H5)

static IRQn_Type exti_irqn(const uint8_t line)
{
    return (IRQn_Type)(EXTI0_IRQn + line);
}

static void exti_clear_pending(const uint32_t bit)
{
    EXTI->RPR1 = bit;
    EXTI->FPR1 = bit;
}

static void exti_dispatch(void* ctx)
{
    const uint32_t line = (uint32_t)(uintptr_t)ctx;
    exti_clear_pending(1UL << line);
    const ExtiSlot* s = &exti_slots[line];
    if (s->handler) s->handler(s->ctx);
}

static uint32_t exti_dispatch_ctx(const uint8_t line)
{
    return line;
}

static void exti_select_port(const uint8_t line, const uint8_t port)
{
    writeField(EXTI->EXTICR[line >> 2], 0xFFU, (line & 3U) * 8U, port);
}

#endif

bool exti_line_free(const PinName pin)
{
    const uint8_t line = STM_PIN(pin);
    return !getBit(exti_used, line) || exti_slots[line].port == STM_PORT(pin);
}

bool exti_attach(const PinName pin, const ExtiEdge edge, const IrqHandler handler,
                 void* ctx, const uint8_t priority)
{
    if (pin == NC || !exti_line_free(pin)) return false;

    const uint8_t  line = STM_PIN(pin);
    const uint8_t  port = STM_PORT(pin);
    const uint32_t bit  = 1UL << line;

    GPIO_TypeDef* gpio = set_GPIO_Port_Clock(port);
    clearField(gpio->MODER, 3U, line * 2U);   /* input, keep existing pull */

    const uint32_t primask = irq_lock();

    clearMask(EXTI->IMR1, bit);
    exti_select_port(line, port);

    if (edge & EXTI_RISING)  setMask(EXTI->RTSR1, bit); else clearMask(EXTI->RTSR1, bit);
    if (edge & EXTI_FALLING) setMask(EXTI->FTSR1, bit); else clearMask(EXTI->FTSR1, bit);

    exti_slots[line].handler = handler;
    exti_slots[line].ctx     = ctx;
    exti_slots[line].port    = port;
    setBit(exti_used, line);

    exti_clear_pending(bit);
    setMask(EXTI->IMR1, bit);

    irq_restore(primask);

    irq_attach(exti_irqn(line), exti_dispatch, (void*)(uintptr_t)exti_dispatch_ctx(line), priority);
    return true;
}

void exti_detach(const PinName pin)
{
    const uint8_t line = STM_PIN(pin);
    if (!getBit(exti_used, line) || exti_slots[line].port != STM_PORT(pin)) return;

    const uint32_t bit = 1UL << line;
    clearMask(EXTI->IMR1, bit);
    clearMask(EXTI->RTSR1, bit);
    clearMask(EXTI->FTSR1, bit);
    exti_clear_pending(bit);

    exti_slots[line].handler = NULL;
    exti_slots[line].ctx     = NULL;
    clearBit(exti_used, line);

#if defined(MCU_FAMILY_G4)
    /* Shared vectors stay attached while another line on them is in use. */
    if (exti_used & exti_dispatch_ctx(line)) return;
#endif
    irq_detach(exti_irqn(line));
}
