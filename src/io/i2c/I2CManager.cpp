#include "I2CManager.h"

void I2CManager::configure(I2CBus bus, const uint32_t sda, const uint32_t scl) {
    const auto busIndex = static_cast<uint8_t>(bus);
    buses[busIndex].setSDA(sda);
    buses[busIndex].setSCL(scl);
    configured[busIndex] = true;
}

void I2CManager::init(I2CBus bus) {
    const auto busIndex = static_cast<uint8_t>(bus);
    if (!configured[busIndex] || initialized[busIndex]) return;
    buses[busIndex].begin();
    initialized[busIndex] = true;
}

TwoWire& I2CManager::getBus(I2CBus bus) {
    return buses[static_cast<uint8_t>(bus)];
}
