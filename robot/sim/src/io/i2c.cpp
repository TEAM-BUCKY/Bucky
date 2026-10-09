// Replacement for hardware/io/i2c/I2CDMA.c. Transactions go to simulated chips attached to the bus
// (sim::io::i2c_attach). Blocking transfers cost their wire time at the configured bus speed;
// DMA reads complete instantly (the firmware may spin on bus->busy without a time hook, like
// i2c_dma_wait(), so a DMA read must never be left pending). A missing chip NACKs: blocking reads
// return 0, DMA reads leave the buffer untouched and busy clear — what the real driver does.
#include "hardware/io/i2c/I2CDMA.h"

#include <map>
#include <vector>

#include "core/world.h"
#include "io/io.h"

namespace {

std::map<const I2C_TypeDef*, std::vector<sim::io::I2cDevice*>>& buses() {
    static std::map<const I2C_TypeDef*, std::vector<sim::io::I2cDevice*>> b;
    return b;
}

sim::io::I2cDevice* find(const I2C_TypeDef* i2c, const uint8_t addr) {
    const auto it = buses().find(i2c);
    if (it == buses().end()) return nullptr;
    for (auto* d : it->second)
        if (d->address() == addr && d->present()) return d;
    return nullptr;
}

uint64_t byte_time_ns(const I2C_TypeDef* i2c) {
    switch (i2c->TIMINGR) {
        case I2C_TIMING_FMP_3M4: return 9ULL * 1'000'000'000ULL / 3'400'000ULL;
        case I2C_TIMING_FMP_1M: return 9ULL * 1'000ULL;
        default: return 9ULL * 2'500ULL;   // 400 kHz
    }
}

void wire_time(const I2C_TypeDef* i2c, const uint32_t bytes) {
    sim::world().hook_delay(bytes * byte_time_ns(i2c));
}

}  // namespace

namespace sim::io {
void i2c_attach(I2C_TypeDef* bus, I2cDevice* dev) { buses()[bus].push_back(dev); }
}  // namespace sim::io

extern "C" {

void i2c_dma_init_raw(I2CDMABus* bus, I2C_TypeDef* i2c, PinName /*sda*/, PinName /*scl*/,
                      DmaChannel* dma_rx, const uint32_t rx_request, const uint32_t timing,
                      uint8_t /*irq_priority*/) {
    bus->i2c = i2c;
    bus->dma_rx = dma_rx;
    bus->rx_request = rx_request;
    bus->busy = false;
    i2c->TIMINGR = timing;
    i2c->CR1 |= 1U;   // PE
    dma_reset(dma_rx);
}

void i2c_dma_read_reg(I2CDMABus* bus, const uint8_t addr, const uint8_t reg, volatile uint8_t* buf,
                      const uint8_t len) {
    auto* dev = find(bus->i2c, addr);
    if (dev == nullptr) return;
    uint8_t tmp[256];
    dev->read_regs(reg, tmp, len);
    for (uint8_t i = 0; i < len; i++) buf[i] = tmp[i];
    bus->busy = false;
}

void i2c_dma_write_reg(const I2CDMABus* bus, const uint8_t addr, const uint8_t reg,
                       const uint8_t value) {
    wire_time(bus->i2c, 3);
    if (auto* dev = find(bus->i2c, addr)) dev->write_reg(reg, value);
}

uint8_t i2c_dma_read_reg_blocking(const I2CDMABus* bus, const uint8_t addr, const uint8_t reg) {
    wire_time(bus->i2c, 4);
    auto* dev = find(bus->i2c, addr);
    if (dev == nullptr) return 0;
    uint8_t v = 0;
    dev->read_regs(reg, &v, 1);
    return v;
}

bool i2c_dma_probe(const I2CDMABus* bus, const uint8_t addr) {
    wire_time(bus->i2c, 1);
    return find(bus->i2c, addr) != nullptr;
}

bool i2c_dma_wait_timeout(const I2CDMABus* bus, const uint32_t timeout_ms) {
    const uint32_t start = millis();
    while (bus->busy) {
        if (millis() - start > timeout_ms) return false;
    }
    return true;
}

}  // extern "C"
