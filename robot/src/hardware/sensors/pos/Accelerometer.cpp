#include <Arduino.h>
#include "Accelerometer.h"
#include "debug.h"
#include "optimizations/bitboard.h"

void Accelerometer::begin(I2CDMABus& busRef) {
    bus = &busRef;

    uint8_t id = 0;
    for (uint8_t attempt = 0; attempt < 6; attempt++) {
        id = readReg(LSM303AGR_WHO_AM_I);
        if (id == 0x33) break;
        delay(20);
    }
    DBG_PRINT_SUBJECT(DEBUG_SUBJ_POSITION, "Accel WHO_AM_I: 0x");
    DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_POSITION, id, HEX);
    if (id != 0x33) {
        ok = false;
        return;
    }

    writeReg(LSM303AGR_CTRL_REG1_A, 0x57);
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

    auto toG = [&](const uint8_t i) {
        const auto raw = static_cast<int16_t>(combineBytes(buf[i + 1], buf[i]));
        DBG_PRINT_SUBJECT(DEBUG_SUBJ_POSITION, "Raw value: ");
        DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_POSITION, raw);
        return (raw >> 4) * MG_PER_LSB * 0.001f;
    };
    ax_g = toG(0);
    ay_g = toG(2);
    az_g = toG(4);
    return true;
}
