//
// Created by koen on 2/28/26.
//

#ifndef BUCKY_COMPASS_H
#define BUCKY_COMPASS_H

#include <Wire.h>

#define LIS2MDL_ADDR 0x1E
#define LIS2MDL_WHO_AM_I_REG 0x4F
#define LIS2MDL_CFG_REG_A 0x60
#define LIS2MDL_CFG_REG_C 0x62
#define LIS2MDL_OUTX_L_REG 0x68

class Compass
{
    private:
        TwoWire* wire = nullptr;
        float heading = 0;
        float startHeading = 0;
        bool hasStartHeading = false;

        void writeReg(uint8_t reg, uint8_t value);
        uint8_t readReg(uint8_t reg);

    public:
        bool init(TwoWire& wire);
        void update();
        float getHeading() const;
        float getOffset() const;
        void reset();
};

#endif //BUCKY_COMPASS_H
