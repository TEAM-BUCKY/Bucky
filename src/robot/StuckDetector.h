#ifndef BUCKY_STUCKDETECTOR_H
#define BUCKY_STUCKDETECTOR_H

#include <cstdint>

#include "motor/MotorDriver.h"

// Detect "robot is commanded to move but wheels don't spin" by sampling
// encoder speeds against the commanded drive + rotation. Hardware-only:
// relies on encoder_get_speed() from io/encoder. Not compiled into the
// desktop simulation (which leaves BrainSensors::stuck_detected = false).
class StuckDetector {
public:
    void reset(uint32_t now_ms);

    // Call once per main-loop iteration. drive is in cmd units (±100),
    // rotation is in cmd units (±35). Returns true while stuck.
    bool update(uint32_t now_ms, VectorXY drive, float rotation);

    bool detected() const { return tripped_; }

private:
    // Thresholds — tuned on-bench. Adjust via the constants below.
    static constexpr float kCmdDriveMin   = 15.0f;   // ~180 mm/s
    static constexpr float kCmdRotationMin = 10.0f;  // deg/s-ish
    static constexpr float kStuckTicksPerSec = 40.0f;
    static constexpr uint32_t kTripAfterMs = 400;
    static constexpr uint32_t kClearAfterMs = 150;

    uint32_t stuckSinceMs_ = 0;  // 0 = not currently meeting the condition
    uint32_t clearSinceMs_ = 0;
    bool tripped_ = false;
};

#endif // BUCKY_STUCKDETECTOR_H
