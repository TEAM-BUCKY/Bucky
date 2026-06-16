#ifndef BUCKY_ACCELEROMETER_H
#define BUCKY_ACCELEROMETER_H

#include <Arduino.h>
#include "io/i2c/I2CDMA.h"

// LSM303AGR accelerometer (paired with LIS2MDL magnetometer on the board).
#define LSM303AGR_ACC_ADDR     0x19
#define LSM303AGR_WHO_AM_I     0x0F    // expected 0x33
#define LSM303AGR_CTRL_REG1_A  0x20
#define LSM303AGR_CTRL_REG4_A  0x23
#define LSM303AGR_OUT_X_L_A    0x28
#define LSM303AGR_AUTOINC_MASK 0x80

class Accelerometer {
    I2CDMABus* bus = nullptr;
    bool ok = false;

    // HR mode (12-bit), ±2g => 0.98 mg/digit after right-shifting the 16-bit frame by 4.
    static constexpr float MG_PER_LSB = 0.98f;

public:
    void begin(I2CDMABus& busRef);
    [[nodiscard]] bool isOk() const { return ok; }

    // Blocking DMA read. Returns false on timeout.
    bool read(float& ax_g, float& ay_g, float& az_g) const;
};

#endif // BUCKY_ACCELEROMETER_H
