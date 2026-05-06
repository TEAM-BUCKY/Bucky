#ifndef BUCKY_IRBALLTRACKER_H
#define BUCKY_IRBALLTRACKER_H

#include <cstdint>
#include "../../sensors/IRSensor.h"

// A single ball estimate produced from the IR sensor ring.
struct IRBallObservation {
    bool valid = false;
    float bearingDeg = 0.0f;
    float rangeCm = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float strength = 0.0f;
    float confidence = 0.0f;
    uint32_t sensorCount = 0;
    uint8_t peakSensor = 0;
};

class IRBallProcessor
{
    public:
        static constexpr uint32_t MAX_SENSORS = 32;
        static constexpr uint32_t LUT_SIZE = 24;

        // Configure sensor calibration and convert raw readings into an observation.
        IRBallProcessor();

        void reset();
        void setThresholdRatio(float ratio);
        void setSensorCalibration(const float* gains, const float* baselines, uint32_t count);
        void setDistanceCalibration(const float* amplitudes, const float* distances, uint32_t count);
        void setDefaultDistanceCalibration(float minDistanceCm = 5.0f, float maxDistanceCm = 200.0f);

        // Physical-sensor-to-buffer-index rotation for the mux. Buffer slot
        // used for sensor i becomes (i + offset) % IR_MUX_CHANNELS. Pair
        // with ir_calibrate_channels() / ir_get_channel_offset().
        // process() also re-detects the offset every frame and auto-corrects
        // if the free-running mux counter phase drifts at runtime.
        void setChannelOffset(uint32_t offset);

        // One-shot workaround for boot calibration misfires: rotate
        // channelOffset_ so the given peakSensor becomes the new sensor 0
        // (front). Intended to be called by the main loop on the first
        // confident IR observation while the robot drives forward blindly —
        // whichever sensor first sees the ball is, by construction, the
        // front. No-op after the first successful call per instance.
        void lockS0AtSensor(uint8_t peakSensor);
        [[nodiscard]] bool isS0Locked() const { return s0Locked_; }

        [[nodiscard]] IRBallObservation process(const uint16_t* raw, uint32_t sensorCount) const;

    private:
        float gains_[MAX_SENSORS] = {};
        float baselines_[MAX_SENSORS] = {};
        float amplitudeLut_[LUT_SIZE] = {};
        float distanceLut_[LUT_SIZE] = {};
        uint32_t lutCount_ = LUT_SIZE;
        // Cached sin/cos for sensor angles 2π·i/n. Lazily rebuilt in process()
        // whenever `n` changes (typically once, at boot).
        mutable float sinAngle_[MAX_SENSORS] = {};
        mutable float cosAngle_[MAX_SENSORS] = {};
        mutable uint32_t cachedSensorCount_ = 0;
        // Low threshold ratio keeps bearing smooth as the ball moves between
        // sensors: neighbor-sensor crosstalk (~5-10% of peak) needs to count,
        // or the weighted centroid collapses to the peak sensor's exact angle.
        float thresholdRatio_ = 0.08f;
        mutable uint32_t channelOffset_ = 0;

        // Runtime mux-phase drift tracking. The offset is only updated after
        // kDriftConfirmFrames consecutive frames agree on a new value — a
        // single ball transient can't flip it. Detection is also gated on
        // peakMax < kDriftDetectPeakMax: with a bright ball present, dark
        // back-facing connected sensors can read lower than floating
        // unconnected mux inputs, which fools the "quietest cluster"
        // heuristic into latching a 180°-rotated offset.
        static constexpr uint32_t kDriftConfirmFrames = 8;
        static constexpr uint16_t kDriftDetectPeakMax = 250;
        mutable uint32_t driftFrames_ = 0;
        mutable uint32_t pendingOffset_ = 0;

        bool s0Locked_ = false;

        static float lookupDistanceFromAmplitude(const float* amplitudes, const float* distances, uint32_t count, float amplitude);
};

#endif // BUCKY_IRBALLTRACKER_H

