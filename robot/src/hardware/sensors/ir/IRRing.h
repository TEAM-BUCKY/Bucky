#ifndef ROBOT_IRRING_H
#define ROBOT_IRRING_H

#include <hardware/sensors/gport/GPort.h>
#include <hardware/motor/Motors.h>

struct IRConfig {
    uint8_t sensors = 12;
    float baselineCode = 4.0f;
    float kCm = 760.0f;
    uint16_t clipCode = 2300;
    float mountRadiusCm = 9.5f;
    float cosPower = 20.0f;
    float minAmp = 8.0f;
    float minNeighbourAmp = 2.0f;
    float maxCm = 300.0f;
};

struct BallReading {
    bool seen = false;
    bool clipped = false;
    float angle = 0.0f;
    float distanceCm = 0.0f;
    float sensorDistanceCm = 0.0f;
    VectorXY position = {0.0f, 0.0f};
    float peakAmp = 0.0f;
    uint8_t peakSensor = 0;
    uint8_t contributors = 0;
    Percentage confidence = 0.0f;
};

class IRRing
{
public:
    explicit IRRing(const GPort& port, const IRConfig& config = {}) : port(port), cfg(config) {}

    [[nodiscard]] bool ready() const { return port.hasBegun() && port.kind() == GSensorKind::IR; }

    bool update();

    [[nodiscard]] const BallReading& ball() const { return last; }

    // Robot frame.
    [[nodiscard]] VectorXY getBallPosition() const { return last.position; }

    [[nodiscard]] const uint16_t* raw() const { return frame; }

    [[nodiscard]] const IRConfig& config() const { return cfg; }

    // Radians.
    static float sensorAngle(uint8_t sensor, const IRConfig& cfg);

    static float sensorDistanceCm(float amp, const IRConfig& cfg);

    static BallReading estimate(const uint16_t frame[GPort::SENSORS], const IRConfig& cfg);

private:
    const GPort& port;
    IRConfig cfg;
    uint32_t lastSeq = 0;
    uint16_t frame[GPort::SENSORS] = {};
    BallReading last = {};
};

#endif //ROBOT_IRRING_H
