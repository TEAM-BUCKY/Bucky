#include "Compass.h"

void Compass::writeReg(uint8_t reg, uint8_t value) {
    wire->beginTransmission(LIS2MDL_ADDR);
    wire->write(reg);
    wire->write(value);
    wire->endTransmission();
}

uint8_t Compass::readReg(uint8_t reg) {
    wire->beginTransmission(LIS2MDL_ADDR);
    wire->write(reg);
    wire->endTransmission(false);
    wire->requestFrom((uint8_t)LIS2MDL_ADDR, (uint8_t)1);
    return wire->read();
}

bool Compass::init(TwoWire& wireRef) {
    wire = &wireRef;

    delay(20);

    // Check WHO_AM_I
    uint8_t id = readReg(LIS2MDL_WHO_AM_I_REG);
    Serial.print("Compass WHO_AM_I: 0x");
    Serial.println(id, HEX);
    if (id != 0x40) {
        return false;
    }

    // Soft reset
    writeReg(LIS2MDL_CFG_REG_A, 0x20);
    delay(100);

    // Configure: continuous mode, temp compensation, 100Hz ODR
    // CFG_REG_A bits: [7] COMP_TEMP_EN=1, [3:2] ODR=11 (100Hz), [1:0] MD=00 (continuous)
    writeReg(LIS2MDL_CFG_REG_A, 0x8C);

    // CFG_REG_C: BDU=0 (continuous update), no other features
    writeReg(LIS2MDL_CFG_REG_C, 0x00);

    // Let the sensor settle
    delay(200);
    for (int i = 0; i < 10; i++) {
        update();
        delay(20);
    }

    startHeading = heading;
    hasStartHeading = true;

    return true;
}

void Compass::update() {
    wire->beginTransmission(LIS2MDL_ADDR);
    wire->write(LIS2MDL_OUTX_L_REG);
    wire->endTransmission(false);

    uint8_t count = wire->requestFrom((uint8_t)LIS2MDL_ADDR, (uint8_t)6);
    if (count < 6) {
        return;
    }

    uint8_t xl = wire->read();
    uint8_t xh = wire->read();
    uint8_t yl = wire->read();
    uint8_t yh = wire->read();
    uint8_t zl = wire->read();
    uint8_t zh = wire->read();

    int16_t rawX = (int16_t)((xh << 8) | xl);
    int16_t rawY = (int16_t)((yh << 8) | yl);

    float x = rawX * 1.5f * 0.1f;
    float y = rawY * 1.5f * 0.1f;

    heading = atan2(y, x) * 180.0f / PI;
    if (heading < 0) heading += 360.0f;

    if (!hasStartHeading) {
        startHeading = heading;
        hasStartHeading = true;
    }
}

float Compass::getHeading() const {
    return heading;
}

float Compass::getOffset() const {
    float diff = heading - startHeading;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

void Compass::reset() {
    startHeading = heading;
}
