#ifndef BUCKY_GPIO_H
#define BUCKY_GPIO_H

#include <Arduino.h>
#include <stm32g4xx.h>

#include "optimizations/bitboard.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t mask;
} GpioPin;

static inline GpioPin gpio_pin_init(int pin)
{
    PinName pn = digitalPinToPinName(pin);
    GpioPin gp;
    gp.port = get_GPIO_Port(STM_PORT(pn));
    gp.mask = 1U << STM_PIN(pn);
    return gp;
}

static inline void gpio_mode(GpioPin gp, int mode)
{
    uint8_t pos = GetLSB(gp.mask);
    writeField(gp.port->MODER, 0x3U, pos * 2, mode == OUTPUT ? 1U : 0U);

    if (mode == INPUT_PULLUP)
        writeField(gp.port->PUPDR, 0x3U, pos * 2, 1U);
    else if (mode == INPUT_PULLDOWN)
        writeField(gp.port->PUPDR, 0x3U, pos * 2, 2U);
    else
        clearField(gp.port->PUPDR, 0x3U, pos * 2);
}

static inline void gpio_write(GpioPin gp, int high)
{
    gp.port->BSRR = high ? gp.mask : (uint32_t)gp.mask << 16;
}

static inline int gpio_read(GpioPin gp)
{
    return (gp.port->IDR & gp.mask) != 0;
}

static inline void gpio_high(GpioPin gp)
{
    gp.port->BSRR = gp.mask;
}

static inline void gpio_low(GpioPin gp)
{
    gp.port->BSRR = (uint32_t)gp.mask << 16;
}

static inline void gpio_toggle(GpioPin gp)
{
    toggleMask(gp.port->ODR, gp.mask);
}

#endif // BUCKY_GPIO_H
