#ifndef ROBOT_EKF_H
#define ROBOT_EKF_H
#include "DigitalField.h"
#include "helpers/Math.h"

// Frames / units used everywhere in the EKF:
//  - field frame: cm, origin as in DigitalField
//  - field frame: x = right, y = forward (towards opponent goal)
//  - angles: radians, clockwise positive, 0 = +y (forward). Direction of angle a = (sin a, cos a)
//  - theta: robot heading in the field frame (clockwise from field +y)
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

    float speedX;
    float speedY;
};

// Standard deviations (1 sigma). Fill in from datasheets / measurements.
struct EKFNoise
{
    float accel = 0.05f * 981.0f; // cm/s^2, LSM303AGR accel incl. vibration (input noise -> Q)
    float thetaDrift = 0.05f;     // rad/sqrt(s), heading random walk (-> Q)
    float ballAccel = 300.0f;     // cm/s^2, unmodelled ball accel: kicks, bounces, friction (-> Q)
    float ballSpeedInit = 100.0f; // cm/s, ball speed uncertainty on first sighting
    float sonarPos = 3.0f;        // cm, x/y derived from sonar (-> R)
    float compassTheta = 0.05f;   // rad, LIS2MDL heading (-> R)
    float encoderSpeed = 5.0f;    // cm/s, field frame speed from encoders (-> R)
    float irAngle = 0.1f;         // rad, IR ball angle (-> R)
    float irDistance = 10.0f;     // cm, IR ball distance (-> R)
};

//  State x = [robotX, robotY, robotTheta, speedX, speedY, ballX, ballY, ballSpeedX, ballSpeedY]
//
//  predict:  accelerometer is the control input u = (ax, ay) in robot frame (g)
//      a_f    = R(theta) * u
//      pos   += v*dt + 1/2 a_f dt^2
//      v     += a_f dt
//      ball  += ballSpeed*dt         (constant velocity, ballAccel as process noise)
//
//  updates (sequential, call whenever a sensor has fresh data):
//      sonar   -> robotX, robotY        H = identity rows
//      compass -> robotTheta            H = identity row (innovation wrapped)
//      encoder -> speedX, speedY        H = identity rows (already field frame)
//      IR      -> angle, distance       h = [atan2(bx-x, by-y) - theta, |b - p|]  (nonlinear)
class EKF
{
    static constexpr std::size_t N = 9;
    enum Index : std::size_t { X = 0, Y = 1, THETA = 2, VX = 3, VY = 4, BX = 5, BY = 6, BVX = 7, BVY = 8 };

    Math::Vector<N> x = {};
    Math::Matrix<N, N> stateCovariance = {};
    EKFNoise noise;
    bool ballInitialized = false;

    State state = {};

    // Scalar Kalman update with measurement Jacobian row h, innovation already computed.
    void updateScalar(const Math::Vector<N>& h, float innovation, float variance);
    void updateIdentity(std::size_t i, float innovation, float variance);
    void syncState();

    public:
    void init(float robotX, float robotY, float robotTheta, float positionStd = 20.0f, float thetaStd = 0.3f);
    void setNoise(const EKFNoise& n) { noise = n; }

    void predict(float accelX_g, float accelY_g, float dt);

    void updatePosition(float robotX, float robotY);
    void updateHeading(float robotTheta);
    void updateSpeed(float speedX, float speedY);
    // IR: ball angle (rad, robot frame, 0 = forward, clockwise positive) + distance (cm)
    void updateBall(float angle, float distance);

    [[nodiscard]] const State& getState() const { return state; }
    [[nodiscard]] const Math::Matrix<N, N>& getCovariance() const { return stateCovariance; }
    void writeTo(DigitalField& field) const;
};


#endif //ROBOT_EKF_H
