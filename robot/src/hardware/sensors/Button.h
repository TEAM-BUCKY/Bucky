#ifndef ROBOT_BUTTON_H
#define ROBOT_BUTTON_H

#include "hardware/io/gpio/gpio.h"
#include "hardware/io/irq/IRQ.h"

class Button {
public:
    bool begin(const PinName pin) {
        gpio = gpio_pin_init(pin);
        gpio_mode(gpio, INPUT_PULLDOWN);   // pressed pulls the pad to 3.3 V
        stable = gpio_read(gpio);
        changed = millis();
        pending = false;
        return exti_attach(pin, EXTI_BOTH, onEdge, this, BUTTON_EXTI_PRIORITY);
    }

    [[nodiscard]] bool pressed() {
        const uint32_t primask = irq_lock();
        if (millis() - changed >= DEBOUNCE_MS) accept(gpio_read(gpio), millis());
        const bool result = pending;
        pending = false;
        irq_restore(primask);
        return result;
    }

private:
    static constexpr uint32_t DEBOUNCE_MS = 25;
    static constexpr uint8_t BUTTON_EXTI_PRIORITY = 6;

    GpioPin gpio{};
    volatile int stable = 0;
    volatile uint32_t changed = 0;
    volatile bool pending = false;

    void accept(const int level, const uint32_t now) {
        if (level == stable) return;
        stable = level;
        changed = now;
        if (level == 1) pending = true;
    }

    static void onEdge(void* ctx) {
        auto* self = static_cast<Button*>(ctx);
        const uint32_t now = millis();
        if (now - self->changed < DEBOUNCE_MS) return;   // contact bounce
        self->accept(gpio_read(self->gpio), now);
    }
};

inline Button button1;
inline Button button2;

#endif //ROBOT_BUTTON_H
