// Replacement for hardware/io/irq/IRQ.c: same API and line-ownership rules, but handlers are
// called by the simulator (World::raise_irq) instead of through a relocated vector table.
#include "hardware/io/irq/IRQ.h"

#include "optimizations/bitboard.h"

#include "core/world.h"
#include "io/io.h"

namespace {

struct IrqSlot {
    IrqHandler handler;
    void* ctx;
};

struct ExtiSlot {
    IrqHandler handler;
    void* ctx;
    uint8_t port;
    bool rising;
    bool falling;
};

IrqSlot irq_slots[MCU_IRQ_COUNT];
ExtiSlot exti_slots[16];
uint16_t exti_used = 0;

void reset_irq() {
    for (auto& s : irq_slots) s = {};
    for (auto& s : exti_slots) s = {};
    exti_used = 0;
}

const bool registered = [] {
    sim::world().reset_hooks.emplace_back(reset_irq);
    return true;
}();

}  // namespace

extern "C" {

void irq_attach(const IRQn_Type irqn, const IrqHandler handler, void* ctx, uint8_t /*priority*/) {
    const auto n = static_cast<uint32_t>(irqn);
    if (n >= MCU_IRQ_COUNT) return;
    irq_slots[n] = {handler, ctx};
}

void irq_detach(const IRQn_Type irqn) {
    const auto n = static_cast<uint32_t>(irqn);
    if (n >= MCU_IRQ_COUNT) return;
    irq_slots[n] = {};
}

bool exti_line_free(const PinName pin) {
    const uint8_t line = STM_PIN(pin);
    return !(exti_used & (1U << line)) || exti_slots[line].port == STM_PORT(pin);
}

bool exti_attach(const PinName pin, const ExtiEdge edge, const IrqHandler handler, void* ctx,
                 uint8_t /*priority*/) {
    if (pin == NC || !exti_line_free(pin)) return false;
    const uint8_t line = STM_PIN(pin);
    const uint8_t port = STM_PORT(pin);

    GPIO_TypeDef* gpio = set_GPIO_Port_Clock(port);
    clearField(gpio->MODER, 3U, line * 2U);

    const uint32_t primask = irq_lock();
    exti_slots[line] = {handler, ctx, port, (edge & EXTI_RISING) != 0, (edge & EXTI_FALLING) != 0};
    exti_used |= 1U << line;
    irq_restore(primask);
    return true;
}

void exti_detach(const PinName pin) {
    const uint8_t line = STM_PIN(pin);
    if (!(exti_used & (1U << line)) || exti_slots[line].port != STM_PORT(pin)) return;
    exti_slots[line] = {};
    exti_used &= ~(1U << line);
}

}  // extern "C"

namespace sim::io {

void exti_edge(const uint32_t port, const uint32_t line, const bool rising) {
    if (line >= 16 || !(exti_used & (1U << line))) return;
    const ExtiSlot& s = exti_slots[line];
    if (s.port != port) return;
    if ((rising && !s.rising) || (!rising && !s.falling)) return;
    world().raise_irq(s.handler, s.ctx);
}

}  // namespace sim::io
