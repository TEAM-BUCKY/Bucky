#include "Compass.h"
#include "debug.h"
#include "../../optimizations/bitboard.h"
#include "../../io/cordic/cordic.h"

void Compass::writeReg(const uint8_t reg, const uint8_t value) const {
    i2c_dma_write_reg(bus, LIS2MDL_ADDR, reg, value);
}

uint8_t Compass::readReg(const uint8_t reg) const {
    return i2c_dma_read_reg_blocking(bus, LIS2MDL_ADDR, reg);
}

void Compass::begin(I2CDMABus& busRef) {
    bus = &busRef;
    state = CompassState::BOOT_WAIT;
    stateStart = millis();
}

bool Compass::tick() {
    switch (state) {
        case CompassState::BOOT_WAIT:
            if (millis() - stateStart >= 20) {
                state = CompassState::CHECK_ID;
            }
            break;

        case CompassState::CHECK_ID: {
            const uint8_t id = readReg(LIS2MDL_WHO_AM_I_REG);
            DBG_PRINT("Compass WHO_AM_I: 0x");
            DBG_PRINTLN(id, HEX);
            if (id != 0x40) {
                state = CompassState::FAILED;
                break;
            }
            // Soft reset
            writeReg(LIS2MDL_CFG_REG_A, 0x20);
            state = CompassState::RESET_WAIT;
            stateStart = millis();
            break;
        }

        case CompassState::RESET_WAIT:
            if (millis() - stateStart >= 100) {
                state = CompassState::CONFIGURE;
            }
            break;

        case CompassState::CONFIGURE:
            writeReg(LIS2MDL_CFG_REG_A, 0x8C);
            writeReg(LIS2MDL_CFG_REG_C, 0x00);
            state = CompassState::SETTLE_WAIT;
            stateStart = millis();
            break;

        case CompassState::SETTLE_WAIT:
            if (millis() - stateStart >= 200) {
                state = CompassState::SAMPLING;
                sampleCount = 0;
                stateStart = millis();
            }
            break;

        case CompassState::SAMPLING:
            if (millis() - stateStart >= 20) {
                update();
                sampleCount++;
                stateStart = millis();
                if (sampleCount >= 10) {
                    startHeading = heading;
                    hasStartHeading = true;
                    state = CompassState::READY;
                }
            }
            break;

        default:
            break;
    }

    return state == CompassState::READY || state == CompassState::FAILED;
}

void Compass::startRead() {
    i2c_dma_read_reg(bus, LIS2MDL_ADDR, LIS2MDL_OUTX_L_REG, rx_buf, 6);
}

bool Compass::isReadComplete() const {
    return !i2c_dma_is_busy(bus);
}

void Compass::processRead() {
    const auto rawX = static_cast<int16_t>(combineBytes(rx_buf[1], rx_buf[0]));
    const auto rawY = static_cast<int16_t>(combineBytes(rx_buf[3], rx_buf[2]));

    // const float x = rawX * 1.5f * 0.1f;
    // const float y = rawY * 1.5f * 0.1f;

    heading = cordic_atan2(rawY, rawX) * 180.0f / PI_F;
    if (heading < 0) heading += 360.0f;

    if (!hasStartHeading) {
        startHeading = heading;
        hasStartHeading = true;
    }
}

void Compass::update() {
    startRead();
    while (!isReadComplete()) {}
    processRead();
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

float Compass::computeRotation(const float targetDegrees) {
    const unsigned long now = millis();
    const auto dtMs = static_cast<float>(now - lastTime);
    const float dtS = dtMs * 0.001f;
    lastTime = now;

    float error = getOffset() - targetDegrees;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    float rotation = 0;
    if (fabsf(error) > deadzone) {
        const float derivative = (dtS > 0) ? (error - lastError) / dtS : 0;
        rotation = constrain(-error * kp - derivative * kd, -maxRotation, maxRotation);
    }
    lastError = error;

    return rotation;
}

void Compass::setPD(const float kp_, const float kd_, const float maxRotation_, const float deadzone_) {
    kp = kp_;
    kd = kd_;
    maxRotation = maxRotation_;
    deadzone = deadzone_;
}

void Compass::reset() {
    startHeading = heading;
    lastError = 0;
    lastTime = millis();
}
