#include "Compass.h"
#include "../bitboard/bitboard.h"
#include "cordic/cordic.h"

void Compass::writeReg(const uint8_t reg, const uint8_t value) const {
    wire->beginTransmission(LIS2MDL_ADDR);
    wire->write(reg);
    wire->write(value);
    wire->endTransmission();
}

uint8_t Compass::readReg(const uint8_t reg) const {
    wire->beginTransmission(LIS2MDL_ADDR);
    wire->write(reg);
    wire->endTransmission(false);
    wire->requestFrom(static_cast<uint8_t>(LIS2MDL_ADDR), static_cast<uint8_t>(1));
    return wire->read();
}

void Compass::begin(TwoWire& wireRef) {
    wire = &wireRef;
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
            Serial.print("Compass WHO_AM_I: 0x");
            Serial.println(id, HEX);
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

void Compass::update() {
    wire->beginTransmission(LIS2MDL_ADDR);
    wire->write(LIS2MDL_OUTX_L_REG);
    wire->endTransmission(false);

    const uint8_t count = wire->requestFrom(static_cast<uint8_t>(LIS2MDL_ADDR), static_cast<uint8_t>(6));
    if (count < 6) {
        return;
    }

    const uint8_t xl = wire->read();
    const uint8_t xh = wire->read();
    const uint8_t yl = wire->read();
    const uint8_t yh = wire->read();
    const uint8_t zl = wire->read();
    const uint8_t zh = wire->read();

    const auto rawX = static_cast<int16_t>(combineBytes(xh, xl));
    const auto rawY = static_cast<int16_t>(combineBytes(yh, yl));

    const float x = rawX * 1.5f * 0.1f;
    const float y = rawY * 1.5f * 0.1f;

    heading = cordicAtan2(y, x) * 180.0f / PI;
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

float Compass::computeRotation(const float targetDegrees) {
    const unsigned long now = millis();
    const auto dtMs = static_cast<float>(now - lastTime);
    const float dtS = dtMs / 1000.0f;
    lastTime = now;

    float error = getOffset() - targetDegrees;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    float rotation = 0;
    if (fabs(error) > deadzone) {
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
