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
        Motor(int in1, int pwm, float speeddiff);
        Motor(int in1, int pwm);
        void move(float speed, float offset);
        void setup();
};



#endif