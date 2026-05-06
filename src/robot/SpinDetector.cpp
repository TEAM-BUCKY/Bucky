#include "SpinDetector.h"

#include <cmath>

#include "helpers/Math.h"

void SpinDetector::reset(const uint32_t now_ms, const float heading_rad) {
    lastHeading_   = heading_rad;
    lastSampleMs_  = now_ms;
    spinSinceMs_   = 0;
    clearSinceMs_  = now_ms;
    tripped_       = false;
}

bool SpinDetector::update(const uint32_t now_ms, const float heading_rad,
                          const float commanded_rotation_cmd) {
    const uint32_t dt_ms = now_ms - lastSampleMs_;
    if (dt_ms < kMinDtMs) return tripped_;

    const float dTheta = Math::wrapRadians(heading_rad - lastHeading_);
    const float omega  = dTheta / (static_cast<float>(dt_ms) * 0.001f);
    lastHeading_  = heading_rad;
    lastSampleMs_ = now_ms;

    const float commanded_rad_s = fabsf(commanded_rotation_cmd) * kCmdToRadS;
    const bool  spinning = fabsf(omega) > kOmegaLimit
                        && commanded_rad_s < kCommandedOkLim;

    if (spinning) {
        if (spinSinceMs_ == 0) spinSinceMs_ = now_ms;
        clearSinceMs_ = 0;
        if (now_ms - spinSinceMs_ >= kTripAfterMs) tripped_ = true;
    } else {
        spinSinceMs_ = 0;
        if (clearSinceMs_ == 0) clearSinceMs_ = now_ms;
        if (tripped_ && now_ms - clearSinceMs_ >= kClearAfterMs) tripped_ = false;
    }

    return tripped_;
}
