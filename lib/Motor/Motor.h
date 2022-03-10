#ifndef MOTOR_H_
#define MOTOR_H_

#include <Arduino.h>
#include <Core.h>
#include <math.h>

class Motor {
    private:
        int in1, pwm;
        float speeddiff;
    public:
        Motor();
        Motor(int in1, int pwm, float speeddiff);
        Motor(int in1, int pwm);
        void test();
        void move(float speed);
        void setup();
};

class MotorControl {
    private:
        Motor m1, m2, m3;
    public:
        MotorControl(Motor m1, Motor m2, Motor m3);
        void forward(float time, float speed);
        void backward(float time, float speed);
        void move(double degrees, int baseSpeed);
};

#endif