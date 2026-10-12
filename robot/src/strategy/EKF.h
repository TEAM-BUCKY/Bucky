#ifndef ROBOT_EKF_H
#define ROBOT_EKF_H
#include <cstdint>

#include "DigitalField.h"
#include "helpers/Math.h"

// Frames / units used everywhere in the EKF:
//  - field frame: cm, origin as in DigitalField
//  - field frame: x = right, y = forward (towards opponent goal)
//  - angles: radians, clockwise positive, 0 = +y (forward). Direction of angle a = (sin a, cos a)
//  - theta: robot heading in the field frame (clockwise from field +y), omega = dtheta/dt
//  - robot frame: x = right, y = forward
struct State
{
    float ballX;
    float ballY;
    float ballSpeedX;
    float ballSpeedY;

    float robotX;
    float robotY;
    float robotTheta;
    float robotOmega;

    float speedX;
    float speedY;
};

// Standard deviations (1 sigma). Fill in from datasheets / measurements.
struct EKFNoise
{
    float accel = 0.05f * 981.0f; // cm/s^2, LSM303AGR accel incl. vibration (input noise -> Q)
    float accelSpinGain = 0.5f;   // s/rad, accel sigma scales by (1 + gain*|omega|): vibration / lever arm residue
    float angularAccel = 20.0f;   // rad/s^2, unmodelled turn-rate changes (-> Q)
    float thetaDrift = 0.01f;     // rad/sqrt(s), small heading random walk on top of omega (-> Q)
    float omegaInit = 1.0f;       // rad/s, turn-rate uncertainty at init
    float ballAccel = 300.0f;     // cm/s^2, unmodelled ball accel: kicks, bounces (-> Q)
    float ballSpeedInit = 100.0f; // cm/s, ball speed uncertainty on first sighting
    float sonarPos = 3.0f;        // cm, x/y derived from sonar (-> R)
    float compassTheta = 0.05f;   // rad, LIS2MDL heading (-> R)
    float encoderSpeed = 5.0f;    // cm/s, robot frame speed from encoders (-> R)
    float encoderOmega = 0.1f;    // rad/s, turn rate from encoders (-> R)
    float irAngle = 0.1f;         // rad, IR ball angle (-> R)
    float irDistance = 5.0f;      // cm, IR ball distance noise floor (-> R)
    float irDistanceRel = 0.2f;   // fraction of distance, IR range is 1/sqrt(amplitude) so error grows with range
};

// Model constants that are not noise.
struct EKFModel
{
    float imuOffsetX = 0.0f;      // cm, accelerometer position relative to rotation centre (robot frame)
    float imuOffsetY = 0.0f;
    float ballFrictionTau = 2.0f; // s, ball speed decays as exp(-dt / tau); <= 0 disables
    float gateChi2 = 9.0f;        // reject a scalar update when innovation^2 / S exceeds this (3 sigma)
    uint8_t maxRejects = 10;      // accept anyway after this many consecutive rejects (recover from divergence)
};

//  State x = [robotX, robotY, robotTheta, omega, speedX, speedY, ballX, ballY, ballSpeedX, ballSpeedY]
//
//  predict:  accelerometer is the control input u = (ax, ay) in robot frame (g)
//      a_r    = u + omega^2 * r_imu  (remove centripetal accel of an off-centre IMU)
//      a_f    = R(theta) * a_r
//      pos   += v*dt + 1/2 a_f dt^2
//      v     += a_f dt
//      theta += omega*dt             (constant turn rate, angularAccel as process noise)
//      ball  += ballSpeed*dt         (ballSpeed decays with ballFrictionTau, ballAccel as process noise)
//
//  updates (sequential, call whenever a sensor has fresh data, innovation gated):
//      sonar   -> robotX, robotY        H = identity rows
//      compass -> robotTheta            H = identity row (innovation wrapped)
//      encoder -> robot frame speed     h = R(theta)^T v  (nonlinear in theta), omega identity row
//      IR      -> angle, distance       h = [atan2(bx-x, by-y) - theta, |b - p|]  (nonlinear)
class EKF
{
    static constexpr std::size_t N = 10;
    enum Index : std::size_t { X = 0, Y = 1, THETA = 2, OMEGA = 3, VX = 4, VY = 5, BX = 6, BY = 7, BVX = 8, BVY = 9 };

    public:
    enum Sensor : uint8_t { SONAR = 0, COMPASS = 1, ENCODER = 2, IR = 3, SENSOR_COUNT = 4 };

    private:
    Math::Vector<N> x = {};
    Math::Matrix<N, N> stateCovariance = {};
    EKFNoise noise;
    EKFModel model;
    bool ballInitialized = false;

    uint8_t consecutiveRejects[SENSOR_COUNT] = {};
    uint32_t rejectCount[SENSOR_COUNT] = {};

    State state = {};

    // Scalar Kalman update with measurement Jacobian row h, innovation already computed.
    // Returns false when the innovation fails the gate.
    bool updateScalar(const Math::Vector<N>& h, float innovation, float variance, Sensor sensor);
    bool updateIdentity(std::size_t i, float innovation, float variance, Sensor sensor);
    void syncState();

    public:
    void init(float robotX, float robotY, float robotTheta, float positionStd = 20.0f, float thetaStd = 0.3f);
    void setNoise(const EKFNoise& n) { noise = n; }
    void setModel(const EKFModel& m) { model = m; }

    void predict(float accelX_g, float accelY_g, float dt);

    void updatePosition(float robotX, float robotY);
    void updateHeading(float robotTheta);
    // Encoders: robot frame speed (cm/s, x = right, y = forward) + turn rate (rad/s, clockwise positive)
    void updateSpeed(float speedX, float speedY, float omega);
    // IR: ball angle (rad, robot frame, 0 = forward, clockwise positive) + distance (cm)
    void updateBall(float angle, float distance);

    [[nodiscard]] const State& getState() const { return state; }
    [[nodiscard]] const Math::Matrix<N, N>& getCovariance() const { return stateCovariance; }
    [[nodiscard]] uint32_t getRejectCount(const Sensor sensor) const { return rejectCount[sensor]; }
    void writeTo(DigitalField& field) const;
};


#endif //ROBOT_EKF_H
