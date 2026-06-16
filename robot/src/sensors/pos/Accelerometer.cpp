#include "Accelerometer.h"
#include "debug.h"

void Accelerometer::begin(I2CDMABus& busRef) {
    bus = &busRef;

    // Cold-boot I2C sometimes returns 0x00 on the first couple of reads.
    uint8_t id = 0;
    for (uint8_t attempt = 0; attempt < 6; attempt++) {
        id = i2c_dma_read_reg_blocking(bus, LSM303AGR_ACC_ADDR, LSM303AGR_WHO_AM_I);
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
    i2c_dma_write_reg(bus, LSM303AGR_ACC_ADDR, LSM303AGR_CTRL_REG1_A, 0x57);
    // BDU=1, ±2g, high-resolution (12-bit).
    i2c_dma_write_reg(bus, LSM303AGR_ACC_ADDR, LSM303AGR_CTRL_REG4_A, 0x88);
    ok = true;
}

bool Accelerometer::read(float& ax_g, float& ay_g, float& az_g) const {
    if (!ok || bus == nullptr) return false;

    volatile uint8_t buf[6] = {};
    i2c_dma_read_reg(bus, LSM303AGR_ACC_ADDR,
                     LSM303AGR_OUT_X_L_A | LSM303AGR_AUTOINC_MASK,
                     buf, 6);

    const uint32_t start = millis();
    while (i2c_dma_is_busy(bus)) {
        if (millis() - start > 10) return false;
    }

    const auto rawX = static_cast<int16_t>(buf[0] | (buf[1] << 8));
    const auto rawY = static_cast<int16_t>(buf[2] | (buf[3] << 8));
    const auto rawZ = static_cast<int16_t>(buf[4] | (buf[5] << 8));

    ax_g = (rawX >> 4) * MG_PER_LSB * 0.001f;
    ay_g = (rawY >> 4) * MG_PER_LSB * 0.001f;
    az_g = (rawZ >> 4) * MG_PER_LSB * 0.001f;
    return true;
}
