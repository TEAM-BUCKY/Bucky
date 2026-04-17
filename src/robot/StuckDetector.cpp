#include "StuckDetector.h"

#include <cmath>

#include "io/encoder/Encoder.h"

void StuckDetector::reset(uint32_t now_ms) {
    stuckSinceMs_ = 0;
    clearSinceMs_ = now_ms;
    tripped_ = false;
}

bool StuckDetector::update(uint32_t now_ms, VectorXY drive, float rotation) {
    const float driveMag = sqrtf(drive.x * drive.x + drive.y * drive.y);
    const bool commanded = driveMag > kCmdDriveMin
                        || fabsf(rotation) > kCmdRotationMin;

    bool wheelsStalled = true;
    for (uint8_t i = 0; i < 3; ++i) {
        if (fabsf(encoder_get_speed(i)) >= kStuckTicksPerSec) {
            wheelsStalled = false;
            break;
        }
    }

    if (commanded && wheelsStalled) {
        if (stuckSinceMs_ == 0) stuckSinceMs_ = now_ms;
        clearSinceMs_ = 0;
        if (now_ms - stuckSinceMs_ >= kTripAfterMs) tripped_ = true;
    } else {
        stuckSinceMs_ = 0;
        if (clearSinceMs_ == 0) clearSinceMs_ = now_ms;
        // Require the all-clear to hold for a short window so one good encoder
        // reading between stalled ones doesn't clear the trip prematurely.
        if (tripped_ && now_ms - clearSinceMs_ >= kClearAfterMs) tripped_ = false;
    }

    return tripped_;
}
