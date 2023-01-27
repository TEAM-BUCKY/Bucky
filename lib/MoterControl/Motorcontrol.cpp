#include <Arduino.h>
#include <Motorcontrol.h>
#include <math.h>

MotorControl::MotorControl(double RobotRadius){
    MotorControl::RobotRadius = RobotRadius;
}

double MotorControl::CalculateTrajectory(double IRBallDegrees, double DistanceToBall)
{
    double X_distance = DistanceToBall * cos(IRBallDegrees);
    double Y_distance = DistanceToBall * sin(IRBallDegrees);
    if (Y_distance - RobotRadius < 0){
        if (X_distance - RobotRadius < 0 < X_distance + RobotRadius){
            return MotorControl::BallBack(X_distance, Y_distance);
        }
        else{
            return MotorControl::BallSideBack(X_distance, Y_distance);
        }
    }
    else {
        return MotorControl::BallFrontal(X_distance, Y_distance);
    }
};

// forward
double MotorControl::BallFrontal(double X_distance, double Y_distance){
    return atan(Y_distance / (2 * X_distance));
};

// Side Back
double MotorControl::BallSideBack(double X_distance, double Y_distance){
    return 180;
};

// backwards
double MotorControl::BallBack(double X_distance, double Y_distance){
    if (X_distance > 0){
        return 270;
    }
    else{
        return 90;
    }
};