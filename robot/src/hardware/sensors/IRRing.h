#ifndef ROBOT_IRRING_H
#define ROBOT_IRRING_H

#include "GPort.h"
#include "hardware/motor/Motors.h"

// IR ball ring: 12 photodiode boards (VEMD2020X01 → TIA → 2× 40 kHz band-pass → peak hold)
// read through the G port mux. Sensor k faces k·360°/sensors clockwise from the front.
//
// Each sensor's held peak follows the square law: N − baseline = (K / d)² · cosⁿ(θ), with d the
// distance from that sensor to the ball and θ the angle off its axis. K = √(P·S·G / 4π) in ADC
// units ≈ 760 cm·√code for the RCJ IR golf ball (P ≈ 3.96 W peak, S = 1.2 µA per W/m²,
// G ≈ 1.21e5 V/A, 3.3 V / 4095). n ≈ 20 fits the VEMD2020's ±15° half sensitivity.
struct IRConfig {
    uint8_t sensors = 12;
    float baselineCode = 4.0f;     // no-ball reading (peak-detector offset ≈ 3 mV)
    float kCm = 760.0f;            // calibrate: K = r_sensor_cm · √(N − baseline) on-axis
    uint16_t clipCode = 2300;      // just under the TIA ceiling (≈ 2387 codes, ≈ 15 cm from a sensor)
    float mountRadiusCm = 11.0f;   // TODO measure: photodiode distance from robot centre
    float cosPower = 20.0f;        // angular response cosⁿ(θ); UNVERIFIED shape
    float minAmp = 8.0f;           // codes above baseline needed to report a ball (≈ 270 cm)
    float minNeighbourAmp = 2.0f;  // codes above baseline for a sensor to join the fit
    float maxCm = 300.0f;
};

struct BallReading {
    bool seen = false;
    bool clipped = false;          // peak sensor saturated: distanceCm is an upper bound
    float angle = 0.0f;            // rad, robot frame, clockwise, 0 = forward, (−π, π]
    float distanceCm = 0.0f;       // from the robot centre (what EKF::updateBall expects)
    float sensorDistanceCm = 0.0f; // from the peak sensor
    VectorXY position = {0.0f, 0.0f};   // cm, x = right, y = forward
    float peakAmp = 0.0f;
    uint8_t peakSensor = 0;
    uint8_t contributors = 0;
    float confidence = 0.0f;       // 0..1
};

class IRRing
{
public:
    explicit IRRing(const GPort& port, const IRConfig& config = {}) : port(port), cfg(config) {}

    [[nodiscard]] bool ready() const { return port.hasBegun() && port.kind() == GSensorKind::IR; }

    // Reads the newest frame if there is one. Returns true when ball() changed.
    bool update();

    [[nodiscard]] const BallReading& ball() const { return last; }
    [[nodiscard]] VectorXY getBallPosition() const { return last.position; }
    [[nodiscard]] const uint16_t* raw() const { return frame; }
    [[nodiscard]] const IRConfig& config() const { return cfg; }

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
