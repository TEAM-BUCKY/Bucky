#ifndef BUCKY_IRBALLTRACKER_H
#define BUCKY_IRBALLTRACKER_H

#include <cstdint>
#include "../sensors/IRSensor.h"

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

class IRBallTracker
{
    public:
        static constexpr uint32_t MAX_SENSORS = 32;
        static constexpr uint32_t LUT_SIZE = 24;

        IRBallTracker();

        void reset();
        void setThresholdRatio(float ratio);
        void setSensorCalibration(const float* gains, const float* baselines, uint32_t count);
        void setDistanceCalibration(const float* amplitudes, const float* distances, uint32_t count);
        void setDefaultDistanceCalibration(float minDistanceCm = 5.0f, float maxDistanceCm = 200.0f);

        [[nodiscard]] IRBallObservation process(const uint16_t* raw, uint32_t sensorCount) const;

    private:
        float gains_[MAX_SENSORS];
        float baselines_[MAX_SENSORS];
        float amplitudeLut_[LUT_SIZE];
        float distanceLut_[LUT_SIZE];
        uint32_t lutCount_ = LUT_SIZE;
        float thresholdRatio_ = 0.30f;

        static float lookupDistanceFromAmplitude(const float* amplitudes, const float* distances, uint32_t count, float amplitude);
};

#endif // BUCKY_IRBALLTRACKER_H

