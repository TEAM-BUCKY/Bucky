// The Arduino core functions the firmware calls, on top of the simulator.
#include <Arduino.h>
#include <HardwareTimer.h>
#include <PeripheralPins.h>
#include <pinmap.h>

#include "optimizations/bitboard.h"
#include <utility/stm32_eeprom.h>

#include <cstdarg>
#include <cstdio>
#include <random>

#include "core/world.h"
#include "io/io.h"

using sim::world;

extern "C" {

// ---- time --------------------------------------------------------------------------------------

void init(void) {}

uint32_t millis(void) {
    world().hook_poll();
    return world().millis32();
}

uint32_t micros(void) {
    world().hook_poll();
    return world().micros32();
}

void delay(const uint32_t ms) { world().hook_delay(static_cast<uint64_t>(ms) * 1'000'000ULL); }

void delayMicroseconds(const uint32_t us) { world().hook_delay(static_cast<uint64_t>(us) * 1'000ULL); }

uint32_t sim_get_primask(void) { return world().primask(); }
void sim_set_primask(const uint32_t primask) { world().set_primask(primask); }

// ---- pins --------------------------------------------------------------------------------------

uint32_t pinNametoDigitalPin(const PinName p) { return static_cast<uint32_t>(p) & PNAME_MASK; }
PinName digitalPinToPinName(const uint32_t pin) { return static_cast<PinName>(pin & PNAME_MASK); }

void pinMode(const uint32_t pin, const uint32_t mode) {
    const PinName p = digitalPinToPinName(pin);
    GPIO_TypeDef* g = set_GPIO_Port_Clock(STM_PORT(p));
    if (g == nullptr) return;
    const uint32_t line = STM_PIN(p);
    uint32_t moder = sim::io::MODE_INPUT, pupd = 0;
    switch (mode) {
        case OUTPUT:
        case OUTPUT_OPEN_DRAIN: moder = sim::io::MODE_OUTPUT; break;
        case INPUT_PULLUP: pupd = 1; break;
        case INPUT_PULLDOWN: pupd = 2; break;
        case INPUT_ANALOG: moder = sim::io::MODE_ANALOG; break;
        default: break;
    }
    writeField(g->MODER, 3U, line * 2, moder);
    writeField(g->PUPDR, 3U, line * 2, pupd);
}

void digitalWriteFast(const PinName pin, const uint32_t value) {
    GPIO_TypeDef* g = set_GPIO_Port_Clock(STM_PORT(pin));
    if (g == nullptr) return;
    const uint32_t mask = 1U << STM_PIN(pin);
    g->BSRR = value ? mask : mask << 16;
}

int digitalReadFast(const PinName pin) {
    GPIO_TypeDef* g = set_GPIO_Port_Clock(STM_PORT(pin));
    return g ? ((g->IDR >> STM_PIN(pin)) & 1U) : 0;
}

void digitalWrite(const uint32_t pin, const uint32_t value) {
    digitalWriteFast(digitalPinToPinName(pin), value);
}

int digitalRead(const uint32_t pin) { return digitalReadFast(digitalPinToPinName(pin)); }

void analogReadResolution(int /*bits*/) {}

uint32_t analogRead(const uint32_t pin) { return sim::io::analog(pin & PNAME_MASK); }

// ---- pinmap (stm32duino SrcWrapper/src/stm32/pinmap.c semantics) -------------------------------

const uint32_t pin_map_ll[16] = {1U << 0, 1U << 1, 1U << 2, 1U << 3, 1U << 4, 1U << 5,
                                 1U << 6, 1U << 7, 1U << 8, 1U << 9, 1U << 10, 1U << 11,
                                 1U << 12, 1U << 13, 1U << 14, 1U << 15};

void pin_function(const PinName pin, const int function) {
    GPIO_TypeDef* g = set_GPIO_Port_Clock(STM_PORT(pin));
    if (g == nullptr) return;
    const uint32_t line = STM_PIN(pin);
    uint32_t moder = sim::io::MODE_INPUT;
    switch (STM_PIN_FUNCTION(function)) {
        case STM_PIN_OUTPUT: moder = sim::io::MODE_OUTPUT; break;
        case STM_PIN_ALTERNATE: moder = sim::io::MODE_AF; break;
        case STM_PIN_ANALOG: moder = sim::io::MODE_ANALOG; break;
        default: break;
    }
    writeField(g->AFR[line >> 3], 0xFU, (line & 7U) * 4U, STM_PIN_AFNUM(function));
    writeField(g->PUPDR, 3U, line * 2, STM_PIN_PUPD(function));
    writeField(g->MODER, 3U, line * 2, moder);
}

bool pin_in_pinmap(const PinName pin, const PinMap* map) {
    for (; map->pin != NC; map++)
        if (map->pin == pin) return true;
    return false;
}

void pinmap_pinout(const PinName pin, const PinMap* map) {
    if (pin == NC) return;
    for (; map->pin != NC; map++) {
        if (map->pin == pin) {
            pin_function(pin, map->function);
            return;
        }
    }
}

void* pinmap_find_peripheral(const PinName pin, const PinMap* map) {
    for (; map->pin != NC; map++)
        if (map->pin == pin) return map->peripheral;
    return nullptr;
}

void* pinmap_peripheral(const PinName pin, const PinMap* map) {
    return pin == NC ? nullptr : pinmap_find_peripheral(pin, map);
}

PinName pinmap_find_pin(void* peripheral, const PinMap* map) {
    for (; map->peripheral != nullptr; map++)
        if (map->peripheral == peripheral) return map->pin;
    return NC;
}

PinName pinmap_pin(void* peripheral, const PinMap* map) {
    return peripheral == nullptr ? NC : pinmap_find_pin(peripheral, map);
}

uint32_t pinmap_find_function(const PinName pin, const PinMap* map) {
    for (; map->pin != NC; map++)
        if (map->pin == pin) return static_cast<uint32_t>(map->function);
    return 0xFFFFFFFFU;
}

uint32_t pinmap_function(const PinName pin, const PinMap* map) {
    return pin == NC ? 0xFFFFFFFFU : pinmap_find_function(pin, map);
}

void* pinmap_merge_peripheral(void* a, void* b) {
    if (a == b) return a;
    if (a == nullptr) return b;
    if (b == nullptr) return a;
    return nullptr;
}

// ---- Print::printf -----------------------------------------------------------------------------

int sim_vdprintf(Print* p, const char* fmt, va_list ap) {
    char stack[256];
    va_list copy;
    va_copy(copy, ap);
    const int n = std::vsnprintf(stack, sizeof stack, fmt, copy);
    va_end(copy);
    if (n < 0) return n;
    if (static_cast<size_t>(n) < sizeof stack) return static_cast<int>(p->write(stack, n));
    std::string big(static_cast<size_t>(n) + 1, '\0');
    std::vsnprintf(big.data(), big.size(), fmt, ap);
    return static_cast<int>(p->write(big.data(), static_cast<size_t>(n)));
}

}  // extern "C"

// ---- WMath -------------------------------------------------------------------------------------

long map(const long x, const long in_min, const long in_max, const long out_min, const long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

namespace {
std::mt19937& rng() {
    static std::mt19937 r(12345);
    return r;
}
}  // namespace

long random(const long howbig) {
    if (howbig == 0) return 0;
    return static_cast<long>(rng()() % static_cast<unsigned long>(howbig));
}

long random(const long howsmall, const long howbig) {
    if (howsmall >= howbig) return howsmall;
    return random(howbig - howsmall) + howsmall;
}

// ---- HardwareTimer (input capture only) --------------------------------------------------------

namespace {
struct Capture {
    TIM_TypeDef* tim;
    uint32_t channel;
    PinName pin;
    callback_function_t cb;
};
std::vector<Capture>& captures() {
    static std::vector<Capture> c;
    return c;
}
const bool hwtimer_registered = [] {
    world().reset_hooks.emplace_back([] { captures().clear(); });
    return true;
}();
}  // namespace

HardwareTimer::HardwareTimer(TIM_TypeDef* instance) : tim(instance) {}
HardwareTimer::~HardwareTimer() = default;

void HardwareTimer::setMode(const uint32_t channel, TimerModes_t /*mode*/, const PinName pin) {
    pinmap_pinout(pin, PinMap_TIM);
    captures().push_back({tim, channel, pin, nullptr});
}

uint32_t HardwareTimer::getTimerClkFreq() { return static_cast<uint32_t>(sim::io::TIMER_CLOCK_HZ); }
void HardwareTimer::setPrescaleFactor(const uint32_t prescaler) { tim->PSC = prescaler - 1U; }
void HardwareTimer::setOverflow(const uint32_t val, TimerFormat_t /*format*/) { tim->ARR = val; }

void HardwareTimer::attachInterrupt(const uint32_t channel, const callback_function_t callback) {
    for (auto& c : captures()) {
        if (c.tim != tim || c.channel != channel) continue;
        c.cb = callback;
        // Latch the 1 MHz counter into CCRn on every edge of the pad, then run the callback.
        TIM_TypeDef* t = tim;
        sim::io::on_input_edge(c.pin, [t, channel, callback](int) {
            *(&t->CCR1 + (channel - 1U)) = world().micros32() & 0xFFFFU;
            world().raise_irq([](void* f) { reinterpret_cast<callback_function_t>(f)(); },
                              reinterpret_cast<void*>(callback));
        });
    }
}

void HardwareTimer::resume() { tim->CR1 |= TIM_CR1_CEN; }
void HardwareTimer::pause() { tim->CR1 &= ~TIM_CR1_CEN; }
