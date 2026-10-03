#ifndef ROBOT_BUTTON_H
#define ROBOT_BUTTON_H

#include "hardware/io/gpio/gpio.h"

class Button {
public:
    void begin(const PinName pin) {
        gpio = gpio_pin_init(pin);
        gpio_mode(gpio, INPUT_PULLDOWN);   // pressed pulls the pad to ground
        level = stable = gpio_read(gpio);
        changed = millis();
    }

    // True once per press, after the level has been settled for the debounce window.
    [[nodiscard]] bool pressed() {
        const int now = gpio_read(gpio);
        if (now != level) {
            level = now;
            changed = millis();
            return false;
        }
        if (now != stable && millis() - changed >= DEBOUNCE_MS) {
            stable = now;
            return now == 0;
        }
        return false;
    }

private:
    static constexpr uint32_t DEBOUNCE_MS = 25;

    GpioPin  gpio{};
    int      level  = 1;
    int      stable = 1;
    uint32_t changed = 0;
};

Button button1;
Button button2;

#endif //ROBOT_BUTTON_H