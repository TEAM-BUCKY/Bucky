#ifndef BUCKY_I2CMANAGER_H
#define BUCKY_I2CMANAGER_H

#include <Wire.h>

enum class I2CBus : uint8_t {
    BUS1 = 0,
    BUS2 = 1,
    BUS3 = 2
};

class I2CManager {
    private:
        TwoWire buses[3];
        bool configured[3] = {};
        bool initialized[3] = {};

    public:
        void configure(I2CBus bus, uint32_t sda, uint32_t scl);
        void init(I2CBus bus);
        TwoWire& getBus(I2CBus bus);
};

#endif //BUCKY_I2CMANAGER_H
