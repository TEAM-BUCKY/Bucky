#ifndef BUCKY_GPIO_H
#define BUCKY_GPIO_H

#include <Arduino.h>

#include "hardware/io/mcu.h"

#include "optimizations/bitboard.h"
#include "optimizations/optimizations.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t mask;
} GpioPin;

static FORCE_INLINE GpioPin gpio_pin_init(const PinName pn)
{
    GpioPin gp;
    gp.port = set_GPIO_Port_Clock(STM_PORT(pn));
    gp.mask = 1U << STM_PIN(pn);
    return gp;
}

static FORCE_INLINE GpioPin gpio_pin_init(const int pin)
{
    return gpio_pin_init(digitalPinToPinName(pin));
}

static FORCE_INLINE void gpio_mode(const GpioPin gp, const int mode)
{
    const uint8_t pos = getLSB(gp.mask);
    writeField(gp.port->MODER, 0x3U, pos * 2, mode == OUTPUT ? 1U : 0U);

    if (mode == INPUT_PULLUP)
        writeField(gp.port->PUPDR, 0x3U, pos * 2, 1U);
    else if (mode == INPUT_PULLDOWN)
        writeField(gp.port->PUPDR, 0x3U, pos * 2, 2U);
    else
        clearField(gp.port->PUPDR, 0x3U, pos * 2);
}

static FORCE_INLINE int gpio_read(const GpioPin gp)
{
    return (gp.port->IDR & gp.mask) != 0;
}

static FORCE_INLINE void gpio_high(const GpioPin gp)
{
    gp.port->BSRR = gp.mask;
}

static FORCE_INLINE void gpio_low(const GpioPin gp)
{
    gp.port->BSRR = static_cast<uint32_t>(gp.mask) << 16;
}

static FORCE_INLINE void gpio_write(const GpioPin gp, const int high)
{
    if (high) gpio_high(gp);
    else      gpio_low(gp);
}

static FORCE_INLINE void gpio_toggle(const GpioPin gp)
{
    toggleMask(gp.port->ODR, gp.mask);
}

static FORCE_INLINE void gpio_hold_low(const PinName pn)
{
    const auto pad = static_cast<PinName>(pn & PNAME_MASK);
    pinMode(pinNametoDigitalPin(pad), OUTPUT);
    digitalWriteFast(pad, LOW);
}

#endif // BUCKY_GPIO_H
