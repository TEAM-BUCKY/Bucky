#ifndef BUCKY_SPINDETECTOR_H
#define BUCKY_SPINDETECTOR_H

#include <cstdint>

// Detect "robot is physically spinning faster than the FSM is asking for"
// by comparing compass-derived omega against the commanded rotation rate.
// Catches compass-PD runaways, wall-wedging spins, and motor failures that
// StuckDetector's "commanded but wheels stalled" check can't see.
//
// Uses the same trip/clear hysteresis pattern as StuckDetector so the shared
// BrainSensors::stuck_detected signal stays coherent — callers OR the two
// detectors together.
class SpinDetector {
public:
    void reset(uint32_t now_ms, float heading_rad);

    // Call once per main-loop iteration. heading_rad is the current compass
    // heading in radians (already wrapped). commanded_rotation is the cmd
    // the motor driver is executing (±35 cmd units). Returns true while
    // spinning uncontrollably.
    bool update(uint32_t now_ms, float heading_rad, float commanded_rotation_cmd);

    bool detected() const { return tripped_; }

private:
    // ~170 °/s. The FSM's rotation cap is ±35 cmd ≈ ±0.7 rad/s (kCmdToRadS =
    // 0.02 in RobotBrain), so 3 rad/s is comfortably above anything the
    // control loop should ever demand.
    static constexpr float kOmegaLimit     = 3.0f;  // rad/s
    static constexpr float kCmdToRadS      = 0.02f; // mirrors RobotBrain
    static constexpr float kCommandedOkLim = 1.0f;  // rad/s — "FSM isn't asking for this"
    static constexpr uint32_t kTripAfterMs = 250;
    static constexpr uint32_t kClearAfterMs = 200;
    // Ignore samples faster than this — dt would underflow noise into omega.
    static constexpr uint32_t kMinDtMs = 3;

    float   lastHeading_ = 0.0f;
    uint32_t lastSampleMs_ = 0;
    uint32_t spinSinceMs_  = 0;
    uint32_t clearSinceMs_ = 0;
    bool     tripped_      = false;
};

#endif // BUCKY_SPINDETECTOR_H
