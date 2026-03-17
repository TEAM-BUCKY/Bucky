#ifndef BUCKY_I2C_DMA_H
#define BUCKY_I2C_DMA_H

#include <stm32g4xx.h>
#include <stdbool.h>
#include <stddef.h>

#include "optimizations/optimizations.h"
#include "optimizations/bitboard.h"

#define DMAMUX_REQ_I2C1_RX  16
#define DMAMUX_REQ_I2C1_TX  17
#define DMAMUX_REQ_I2C2_RX  18
#define DMAMUX_REQ_I2C2_TX  19
#define DMAMUX_REQ_I2C3_RX  20
#define DMAMUX_REQ_I2C3_TX  21

#define I2C_TIMING_FM_400K  0x4052193AU

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    I2C_TypeDef*         i2c;
    DMA_Channel_TypeDef* dma_rx;
    DMA_TypeDef*         dma;
    uint32_t             ifcr_mask;
    uint32_t             rxdr_addr;
    volatile bool        busy;
} I2CDMABus;

void i2c_dma_init(I2CDMABus* bus, I2C_TypeDef* i2c,
                   GPIO_TypeDef* sda_port, uint8_t sda_pin, uint8_t sda_af,
                   GPIO_TypeDef* scl_port, uint8_t scl_pin, uint8_t scl_af,
                   DMA_TypeDef* dma, DMA_Channel_TypeDef* dma_rx,
                   DMAMUX_Channel_TypeDef* dma_mux_rx,
                   uint32_t mux_rx, IRQn_Type dma_rx_irqn);

void i2c_dma_read_reg(I2CDMABus* bus, uint8_t addr, uint8_t reg,
                       volatile uint8_t* buf, uint8_t len);

void i2c_dma_write_reg(const I2CDMABus* bus, uint8_t addr, uint8_t reg, uint8_t value);

uint8_t i2c_dma_read_reg_blocking(const I2CDMABus* bus, uint8_t addr, uint8_t reg);

bool i2c_dma_probe(const I2CDMABus* bus, uint8_t addr);

static FORCE_INLINE bool i2c_dma_is_busy(const I2CDMABus* bus) {
    return bus->busy;
}

static FORCE_INLINE void i2c_dma_wait(const I2CDMABus* bus) {
    while (bus->busy) {}
}

/* Call from the DMA RX transfer-complete ISR. */
static FORCE_INLINE void i2c_dma_rx_isr(I2CDMABus* bus) {
    bus->dma->IFCR   = bus->ifcr_mask;
    bus->dma_rx->CCR = 0;                              /* direct disable, no RMW */
    clearMask(bus->i2c->CR1, I2C_CR1_RXDMAEN);
    __DMB();
    bus->busy = false;
}

#define I2C_DMA_RX_HANDLER(dma_n, ch, bus) \
    void DMA##dma_n##_Channel##ch##_IRQHandler(void) { \
        i2c_dma_rx_isr(&(bus)); \
    }

#ifdef __cplusplus
}
#endif

#endif /* BUCKY_I2C_DMA_H */
