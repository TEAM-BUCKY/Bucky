// GPIO ports: RAM register blocks plus the ODR/BSRR write hooks that turn firmware pin writes
// into edges for the device models, and the input side (set_input → IDR → EXTI).
#include <cstring>
#include <map>
#include <vector>

#include "core/world.h"
#include "io/io.h"

extern "C" {
GPIO_TypeDef sim_GPIOA, sim_GPIOB, sim_GPIOC, sim_GPIOD, sim_GPIOE, sim_GPIOF, sim_GPIOG, sim_GPIOH,
    sim_GPIOI;
GPIO_TypeDef* GPIOPort_list[] = {&sim_GPIOA, &sim_GPIOB, &sim_GPIOC, &sim_GPIOD, &sim_GPIOE,
                                 &sim_GPIOF, &sim_GPIOG, &sim_GPIOH, &sim_GPIOI};

GPIO_TypeDef* set_GPIO_Port_Clock(const uint32_t port_idx) {
    return port_idx < sim::io::PORT_COUNT ? GPIOPort_list[port_idx] : nullptr;
}
}

namespace sim::io {

namespace {
std::map<uint32_t, std::vector<OutputListener>>& listeners() {
    static std::map<uint32_t, std::vector<OutputListener>> l;
    return l;
}

std::map<uint32_t, std::vector<OutputListener>>& input_listeners() {
    static std::map<uint32_t, std::vector<OutputListener>> l;
    return l;
}

std::map<uint32_t, uint16_t> analog_values;

// Pads the outside world drives (survive firmware writes to IDR, which never happen).
uint32_t external_idr[PORT_COUNT] = {};

void reset_ports() {
    for (GPIO_TypeDef* p : GPIOPort_list) std::memset(static_cast<void*>(p), 0, sizeof(*p));
    // Inputs the devices hold (button levels, echo lines) stay where they are across a reboot.
    for (uint32_t i = 0; i < PORT_COUNT; i++) GPIOPort_list[i]->IDR = external_idr[i];
    input_listeners().clear();
}

const bool registered = [] {
    world().reset_hooks.emplace_back(reset_ports);
    return true;
}();

void odr_changed(const int port_idx, const uint32_t old_v, const uint32_t new_v) {
    if (port_idx < 0) return;
    GPIO_TypeDef* p = GPIOPort_list[port_idx];
    uint32_t changed = (old_v ^ new_v) & 0xFFFFU;
    while (changed) {
        const uint32_t line = __builtin_ctz(changed);
        changed &= changed - 1;
        if (((p->MODER >> (line * 2)) & 3U) != MODE_OUTPUT) continue;
        const int level = (new_v >> line) & 1U;
        if (level) p->IDR |= 1U << line; else p->IDR &= ~(1U << line);
        const auto it = listeners().find((port_idx << 4) | line);
        if (it == listeners().end()) continue;
        for (auto& cb : it->second) cb(level);
    }
}
}  // namespace

GPIO_TypeDef* port(const uint32_t index) { return index < PORT_COUNT ? GPIOPort_list[index] : nullptr; }

int port_index(const volatile void* reg) {
    const auto a = reinterpret_cast<uintptr_t>(reg);
    for (uint32_t i = 0; i < PORT_COUNT; i++) {
        const auto base = reinterpret_cast<uintptr_t>(GPIOPort_list[i]);
        if (a >= base && a < base + sizeof(GPIO_TypeDef)) return static_cast<int>(i);
    }
    return -1;
}

uint32_t pin_mode(const PinName p) {
    GPIO_TypeDef* g = port(STM_PORT(p));
    return g ? (g->MODER >> (STM_PIN(p) * 2)) & 3U : MODE_INPUT;
}

uint32_t pin_af(const PinName p) {
    GPIO_TypeDef* g = port(STM_PORT(p));
    const uint32_t line = STM_PIN(p);
    return g ? (g->AFR[line >> 3] >> ((line & 7U) * 4)) & 0xFU : 0;
}

int odr_level(const PinName p) {
    GPIO_TypeDef* g = port(STM_PORT(p));
    return g ? (static_cast<uint32_t>(g->ODR) >> STM_PIN(p)) & 1U : 0;
}

int input_level(const PinName p) {
    GPIO_TypeDef* g = port(STM_PORT(p));
    return g ? (g->IDR >> STM_PIN(p)) & 1U : 0;
}

void set_input(const PinName p, const int level) {
    const uint32_t pi = STM_PORT(p), line = STM_PIN(p);
    GPIO_TypeDef* g = port(pi);
    if (g == nullptr) return;
    const uint32_t bit = 1U << line;
    if (level) external_idr[pi] |= bit; else external_idr[pi] &= ~bit;
    if (pin_mode(p) == MODE_OUTPUT) return;   // the firmware is driving this pad itself
    const bool was = (g->IDR & bit) != 0;
    if (was == (level != 0)) return;
    if (level) g->IDR |= bit; else g->IDR &= ~bit;
    const auto it = input_listeners().find(pad(p));
    if (it != input_listeners().end())
        for (auto& cb : it->second) cb(level);
    exti_edge(pi, line, level != 0);
}

void on_input_edge(const PinName p, OutputListener cb) {
    input_listeners()[pad(p)].push_back(std::move(cb));
}

void set_analog(const uint32_t pin, const uint16_t value) { analog_values[pin & PNAME_MASK] = value; }

uint16_t analog(const uint32_t pin) {
    const auto it = analog_values.find(pin & PNAME_MASK);
    return it == analog_values.end() ? 0 : it->second;
}

void on_output(const PinName p, OutputListener cb) {
    listeners()[pad(p)].push_back(std::move(cb));
}

}  // namespace sim::io

extern "C" void sim_gpio_odr_written(volatile void* reg, const uint32_t old_v, const uint32_t new_v) {
    sim::io::odr_changed(sim::io::port_index(reg), old_v, new_v);
}

extern "C" void sim_gpio_bsrr_written(volatile void* reg, const uint32_t value) {
    const int pi = sim::io::port_index(reg);
    if (pi < 0) return;
    GPIO_TypeDef* g = GPIOPort_list[pi];
    const uint32_t old_v = g->ODR.v;
    const uint32_t set = value & 0xFFFFU, reset = value >> 16;
    const uint32_t new_v = (old_v & ~reset) | set;   // set wins when both bits are written
    g->ODR.v = new_v;
    sim::io::odr_changed(pi, old_v, new_v);
}
