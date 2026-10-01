#ifndef BUCKY_I2C_DMA_H
#define BUCKY_I2C_DMA_H

#include <stdbool.h>
#include <PinNames.h>

#include "hardware/io/mcu.h"
#include "hardware/io/dma/DMA.h"
#include "optimizations/optimizations.h"
#include "optimizations/bitboard.h"

// TIMINGR values depend on the I2C input clock (PCLK, no divider on either board).
#if defined(MCU_FAMILY_G4)
/* 170 MHz */
#define I2C_TIMING_FM_400K  0x4052193AU
#define I2C_TIMING_FMP_1M   0x00805054U
#define I2C_TIMING_FMP_3M4  0x0020131BU
#elif defined(MCU_FAMILY_H5)
/* 250 MHz: the G4 values with each phase rescaled to the same duration.
 * Not yet verified on a scope. */
#define I2C_TIMING_FM_400K  0x60521A3DU
#define I2C_TIMING_FMP_1M   0x00C0767CU
#define I2C_TIMING_FMP_3M4  0x00301C28U
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    I2C_TypeDef*  i2c;
    DmaChannel*   dma_rx;
    uint32_t      rx_request;
    volatile bool busy;
} I2CDMABus;

// SDA/SCL alternate functions are looked up from the core pin map. The RX DMA
// complete interrupt is registered through irq_attach(), no handler macro needed.
void i2c_dma_init_raw(I2CDMABus* bus, I2C_TypeDef* i2c, PinName sda, PinName scl,
                      DmaChannel* dma_rx, uint32_t rx_request,
                      uint32_t timing, uint8_t irq_priority);

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

#ifdef __cplusplus
}

enum class I2CFrequency { FM_400K, FMP_1M, FMP_3M4 };

template<I2CFrequency Freq = I2CFrequency::FM_400K>
FORCE_INLINE void i2c_dma_init(I2CDMABus* bus, I2C_TypeDef* i2c, const PinName sda, const PinName scl,
                               DmaChannel* dma_rx, const uint32_t rx_request,
                               const uint8_t irq_priority = 2)
{
    constexpr uint32_t timing = (Freq == I2CFrequency::FMP_3M4) ? I2C_TIMING_FMP_3M4
                               : (Freq == I2CFrequency::FMP_1M)  ? I2C_TIMING_FMP_1M
                                                                  : I2C_TIMING_FM_400K;
    i2c_dma_init_raw(bus, i2c, sda, scl, dma_rx, rx_request, timing, irq_priority);
}

#endif

#endif /* BUCKY_I2C_DMA_H */
