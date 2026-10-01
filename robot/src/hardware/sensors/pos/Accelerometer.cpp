#include <Arduino.h>
#include "Accelerometer.h"
#include "debug.h"
#include "optimizations/bitboard.h"

void Accelerometer::begin(I2CDMABus& busRef) {
    bus = &busRef;

    // Cold-boot I2C sometimes returns 0x00 on the first couple of reads.
    uint8_t id = 0;
    for (uint8_t attempt = 0; attempt < 6; attempt++) {
        id = readReg(LSM303AGR_WHO_AM_I);
        if (id == 0x33) break;
        delay(20);
    }
    DBG_PRINT("Accel WHO_AM_I: 0x");
    DBG_PRINTLN(id, HEX);
    if (id != 0x33) {
        ok = false;
        return;
    }

    // 100 Hz, X/Y/Z enabled, normal power mode.
    writeReg(LSM303AGR_CTRL_REG1_A, 0x57);
    // BDU=1, ±2g, high-resolution (12-bit).
    writeReg(LSM303AGR_CTRL_REG4_A, 0x88);
    ok = true;
}

bool Accelerometer::read(float& ax_g, float& ay_g, float& az_g) const {
    if (!ok || bus == nullptr) return false;

    volatile uint8_t buf[6] = {};
    i2c_dma_read_reg(bus, LSM303AGR_ACC_ADDR,
                     LSM303AGR_OUT_X_L_A | LSM303AGR_AUTOINC_MASK,
                     buf, 6);

    if (!i2c_dma_wait_timeout(bus, 10)) return false;

    // Left-justified 12-bit samples, little endian.
    auto toG = [&](const uint8_t i) {
        const auto raw = static_cast<int16_t>(combineBytes(buf[i + 1], buf[i]));
        return (raw >> 4) * MG_PER_LSB * 0.001f;
    };
    ax_g = toG(0);
    ay_g = toG(2);
    az_g = toG(4);
    return true;
}
