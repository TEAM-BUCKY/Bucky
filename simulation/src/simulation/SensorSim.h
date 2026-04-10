#ifndef BUCKY_SENSORSIM_H
#define BUCKY_SENSORSIM_H

#include "GroundTruth.h"
#include <random>

enum NoiseLevel { NOISE_NONE = 0, NOISE_LOW, NOISE_HIGH };

struct SimulatedSensors {
    // Compass
    float compassRad = 0.0f;
    bool compassReady = false;

    // Sonar (4 sensors)
    float sonarDistanceM[4] = {};
    bool sonarReady = false;

    // IR ball observation (direct — bypasses IRBallProcessor)
    bool ballVisible = false;
    float ballFieldX = 0.0f;
    float ballFieldY = 0.0f;
    float ballRangeM = 0.0f;
};

class SensorSim {
public:
    SensorSim();

    void setNoiseLevel(NoiseLevel level);
    NoiseLevel getNoiseLevel() const { return noiseLevel_; }

    SimulatedSensors simulate(const GroundTruth& truth, float dt);

private:
    NoiseLevel noiseLevel_ = NOISE_LOW;
    std::mt19937 rng_;
    float compassTimer_ = 0.0f;
    float sonarTimer_ = 0.0f;
    float irTimer_ = 0.0f;

    float gaussian(float stddev);
    float rayCastToWall(float rx, float ry, float angle) const;
};

#endif // BUCKY_SENSORSIM_H
