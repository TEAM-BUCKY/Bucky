//
// Created by koen on 2/28/26.
//

#ifndef BUCKY_COMPASS_H
#define BUCKY_COMPASS_H

#include <Arduino.h>
#include "io/i2c/I2CDMA.h"

#define LIS2MDL_ADDR 0x1E
#define LIS2MDL_WHO_AM_I_REG 0x4F
#define LIS2MDL_CFG_REG_A 0x60
#define LIS2MDL_CFG_REG_C 0x62
#define LIS2MDL_OUTX_L_REG 0x68

enum class CompassState : uint8_t {
    UNINIT,
    BOOT_WAIT,
    CHECK_ID,
    RESET_WAIT,
    CONFIGURE,
    SETTLE_WAIT,
    SAMPLING,
    READY,
    FAILED
};

class Compass
{
        I2CDMABus* bus = nullptr;
        volatile uint8_t rx_buf[6] = {};
        float heading = 0;
        float startHeading = 0;
        bool hasStartHeading = false;

        int16_t lastRawX = 0;
        int16_t lastRawY = 0;
        int16_t lastRawZ = 0;

        float offX = 0, offY = 0, offZ = 0;
        float scaleX = 1, scaleY = 1, scaleZ = 1;

        CompassState state = CompassState::UNINIT;
        uint32_t stateStart = 0;
        uint8_t sampleCount = 0;
        uint8_t whoAmIRetries = 0;

        // PD control state
        float lastError = 0;
        float lastDerivative = 0;     // dirty-D filtered derivative
        unsigned long lastTime = 0;

        float kp = 0.4f;
        float kd = 0.3f;
        float maxRotation = 25.0f;
        float deadzone = 3.0f;
        // Time constant for dirty-D filter. tau = 1/(2*pi*fc); fc ~= 20 Hz.
        float derivTau = 0.00796f;

        void writeReg(const uint8_t reg, const uint8_t value) const { i2c_dma_write_reg(bus, LIS2MDL_ADDR, reg, value); }
        [[nodiscard]] uint8_t readReg(const uint8_t reg) const { return i2c_dma_read_reg_blocking(bus, LIS2MDL_ADDR, reg); }

    public:
        void begin(I2CDMABus& busRef);
        bool tick();
        [[nodiscard]] bool isReady() const { return state == CompassState::READY; }
        [[nodiscard]] bool isFailed() const { return state == CompassState::FAILED; }

        bool update(uint32_t timeoutMs = 10);  // blocking with timeout; true on success
        void startRead() { i2c_dma_read_reg(bus, LIS2MDL_ADDR, LIS2MDL_OUTX_L_REG, rx_buf, 6); }
        [[nodiscard]] bool isReadComplete() const { return !i2c_dma_is_busy(bus);}
        void processRead();            // convert rx_buf into heading

        [[nodiscard]] float getHeading() const { return heading; }
        [[nodiscard]] float getOffset() const;
        [[nodiscard]] int16_t getRawX() const { return lastRawX; }
        [[nodiscard]] int16_t getRawY() const { return lastRawY; }
        [[nodiscard]] int16_t getRawZ() const { return lastRawZ; }
        void setCalibration(float ox, float oy, float oz, float sx, float sy, float sz);
        float computeRotation(float targetDegrees);
        void setPD(float kp, float kd, float maxRotation, float deadzone);
        void reset();
};

#endif //BUCKY_COMPASS_H
