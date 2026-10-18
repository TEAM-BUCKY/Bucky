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

    /**
     * Checks if the IR ring is ready for use.
     * @return true if the IR ring is ready, false otherwise.
     */
    [[nodiscard]] bool ready() const { return port.hasBegun() && port.kind() == GSensorKind::IR; }

    /**
     * Updates the IR ring reading.
     * @return true if a new reading was obtained, false otherwise.
     */
    bool update();

    /**
     * @return the last ball reading.
     */
    [[nodiscard]] const BallReading& ball() const { return last; }

    /**
     * @return the last ball position in robot coordinates.
     */
    [[nodiscard]] VectorXY getBallPosition() const { return last.position; }

    /**
     * @return the last ball distance in centimeters.
     */
    [[nodiscard]] const uint16_t* raw() const { return frame; }

    /**
     * @return the IR ring configuration.
     */
    [[nodiscard]] const IRConfig& config() const { return cfg; }

    /**
     * Calculates the angle of a given sensor in radians, based on its index and the IR configuration.
     * @param sensor The index of the sensor.
     * @param cfg The IR configuration.
     * @return The angle of the sensor in radians.
     */
    static float sensorAngle(uint8_t sensor, const IRConfig& cfg);

    /**
     * Calculates the distance in centimeters based on the amplitude reading and the IR configuration.
     * @param amp The amplitude reading from the sensor.
     * @param cfg The IR configuration.
     * @return The estimated distance in centimeters.
     */
    static float sensorDistanceCm(float amp, const IRConfig& cfg);

    /**
     * Estimates the ball reading based on the IR frame and configuration.
     * @param frame The IR frame.
     * @param cfg The IR configuration.
     * @return The estimated ball reading.
     */
    static BallReading estimate(const uint16_t frame[GPort::SENSORS], const IRConfig& cfg);

private:
    const GPort& port;
    IRConfig cfg;
    uint32_t lastSeq = 0;
    uint16_t frame[GPort::SENSORS] = {};
    BallReading last = {};
};

#endif //ROBOT_IRRING_H
