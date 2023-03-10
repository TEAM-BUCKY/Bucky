#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

class MotorControl {
    private:
        double RobotRadius;
        double BallFrontal(double X_distance, double Y_distance);
        double BallSideBack(double X_distance, double Y_distance);
        double BallBack(double X_distance, double Y_distance);
    public:
        MotorControl(double RobotRadius);
        double CalculateTrajectory(double IRBallDegrees, double DistanceToBall);
};

#endif
