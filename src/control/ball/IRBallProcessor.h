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
        void setChannelOffset(uint32_t offset);

        [[nodiscard]] IRBallObservation process(const uint16_t* raw, uint32_t sensorCount) const;

    private:
        float gains_[MAX_SENSORS] = {};
        float baselines_[MAX_SENSORS] = {};
        float amplitudeLut_[LUT_SIZE] = {};
        float distanceLut_[LUT_SIZE] = {};
        uint32_t lutCount_ = LUT_SIZE;
        // Low threshold ratio keeps bearing smooth as the ball moves between
        // sensors: neighbor-sensor crosstalk (~5-10% of peak) needs to count,
        // or the weighted centroid collapses to the peak sensor's exact angle.
        float thresholdRatio_ = 0.08f;
        uint32_t channelOffset_ = 0;

        static float lookupDistanceFromAmplitude(const float* amplitudes, const float* distances, uint32_t count, float amplitude);
};

#endif // BUCKY_IRBALLTRACKER_H

