#include "I2CManager.h"

void I2CManager::configure(I2CBus bus, uint32_t sda, uint32_t scl) {
    auto i = static_cast<uint8_t>(bus);
    buses[i].setSDA(sda);
    buses[i].setSCL(scl);
    configured[i] = true;
}

void I2CManager::init(I2CBus bus) {
    auto i = static_cast<uint8_t>(bus);
    if (!configured[i] || initialized[i]) return;
    buses[i].begin();
    initialized[i] = true;
}

TwoWire& I2CManager::getBus(I2CBus bus) {
    return buses[static_cast<uint8_t>(bus)];
}
