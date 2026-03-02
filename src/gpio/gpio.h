#ifndef BUCKY_GPIO_H
#define BUCKY_GPIO_H

#include <Arduino.h>
#include <stm32g4xx.h>

struct GpioPin {
    GPIO_TypeDef* port;
    uint16_t mask; // bit mask (1 << pin_number_within_port)

    explicit GpioPin(const int pin) {
        const PinName pn = digitalPinToPinName(pin);
        port = get_GPIO_Port(STM_PORT(pn));
        mask = 1U << STM_PIN(pn);
    }

    constexpr GpioPin(GPIO_TypeDef* port, const uint16_t mask) : port(port), mask(mask) {}
};

inline void gpioMode(const GpioPin gp, const int mode) {
    const uint8_t pos = __builtin_ctz(gp.mask);
    gp.port->MODER = (gp.port->MODER & ~(0x3U << (pos * 2)))
                   | (static_cast<uint32_t>(mode == OUTPUT ? 0x1U : 0x0U) << (pos * 2));

    if (mode == INPUT_PULLUP) {
        gp.port->PUPDR = (gp.port->PUPDR & ~(0x3U << (pos * 2)))
                       | (0x1U << (pos * 2));
    } else if (mode == INPUT_PULLDOWN) {
        gp.port->PUPDR = (gp.port->PUPDR & ~(0x3U << (pos * 2)))
                       | (0x2U << (pos * 2));
    } else {
        gp.port->PUPDR &= ~(0x3U << (pos * 2)); // no pull
    }
}

inline void gpioWrite(const GpioPin gp, const bool high) {
    gp.port->BSRR = high ? gp.mask : (static_cast<uint32_t>(gp.mask) << 16);
}

inline bool gpioRead(const GpioPin gp) {
    return (gp.port->IDR & gp.mask) != 0;
}

inline void gpioHigh(const GpioPin gp) {
    gp.port->BSRR = gp.mask;
}

inline void gpioLow(const GpioPin gp) {
    gp.port->BSRR = static_cast<uint32_t>(gp.mask) << 16;
}

inline void gpioToggle(const GpioPin gp) {
    gp.port->ODR ^= gp.mask;
}

#endif //BUCKY_GPIO_H
