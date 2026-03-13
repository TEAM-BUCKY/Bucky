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
        TwoWire* wire = nullptr;
        float heading = 0;
        float startHeading = 0;
        bool hasStartHeading = false;

        CompassState state = CompassState::UNINIT;
        uint32_t stateStart = 0;
        uint8_t sampleCount = 0;

        // PD control state
        float lastError = 0;
        unsigned long lastTime = 0;

        float kp = 0.4f;
        float kd = 0.3f;
        float maxRotation = 25.0f;
        float deadzone = 3.0f;

        void writeReg(uint8_t reg, uint8_t value) const;
        uint8_t readReg(uint8_t reg) const;

    public:
        void begin(TwoWire& wireRef);
        bool tick();
        bool isReady() const { return state == CompassState::READY; }
        bool isFailed() const { return state == CompassState::FAILED; }
        void update();
        float getHeading() const;
        float getOffset() const;
        float computeRotation(float targetDegrees);
        void setPD(float kp, float kd, float maxRotation, float deadzone);
        void reset();
};

#endif //BUCKY_COMPASS_H
