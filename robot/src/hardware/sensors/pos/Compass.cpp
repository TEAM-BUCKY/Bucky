#include "Compass.h"
#include "debug.h"
#include "optimizations/bitboard.h"
#include "hardware/io/cordic/cordic.h"
#include "helpers/Math.h"

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
            DBG_PRINT_SUBJECT(DEBUG_SUBJ_POSITION, "Compass WHO_AM_I: 0x");
            DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_POSITION, id, HEX);
            if (id != 0x40) {
                if (whoAmIRetries < 5) {
                    whoAmIRetries++;
                    state = CompassState::BOOT_WAIT;
                    stateStart = millis();
                    break;
                }
                state = CompassState::FAILED;
                break;
            }

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
                    lastTime = millis();
                    lastError = 0;
                    lastDerivative = 0;
                    state = CompassState::READY;
                }
            }
            break;

        default:
            break;
    }

    return state == CompassState::READY || state == CompassState::FAILED;
}

void Compass::processRead() {
    lastRawX = static_cast<int16_t>(combineBytes(rx_buf[1], rx_buf[0]));
    lastRawY = static_cast<int16_t>(combineBytes(rx_buf[3], rx_buf[2]));
    lastRawZ = static_cast<int16_t>(combineBytes(rx_buf[5], rx_buf[4]));

    const float cx = (static_cast<float>(lastRawX) - offX) * scaleX;
    const float cy = (static_cast<float>(lastRawY) - offY) * scaleY;

    heading = Math::radiansToDegrees(cordic_atan2(cy, cx));
    if (heading < 0) heading += 360.0f;
}

void Compass::setCalibration(const float ox, const float oy, const float oz,
                             const float sx, const float sy, const float sz) {
    offX = ox; offY = oy; offZ = oz;
    scaleX = sx; scaleY = sy; scaleZ = sz;
}

bool Compass::update(const uint32_t timeoutMs) {
    startRead();
    if (!i2c_dma_wait_timeout(bus, timeoutMs)) return false;
    processRead();
    return true;
}

float Compass::getOffset() const {
    return Math::wrapSignedDegrees(heading - startHeading);
}

float Compass::computeRotation(const float targetDegrees) {
    const unsigned long now = millis();
    const auto dtMs = static_cast<float>(now - lastTime);
    const float dtS = dtMs * 0.001f;
    lastTime = now;

    const float error = Math::wrapSignedDegrees(getOffset() - targetDegrees);

    if (dtS > 0.0f) {
        const float rawDeriv = (error - lastError) / dtS;
        const float alpha    = dtS / (derivTau + dtS);
        lastDerivative      += alpha * (rawDeriv - lastDerivative);
    }

    float rotation = 0;
    if (fabsf(error) > deadzone) {
        rotation = constrain(-error * kp - lastDerivative * kd, -maxRotation, maxRotation);
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
    lastDerivative = 0;
    lastTime = millis();
}
