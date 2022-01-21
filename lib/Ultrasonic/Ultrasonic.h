// define
#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

// include
#include <Arduino.h>
#include <Core.h>


class Ultrasonic {
    private:
        int in1, in2; // in1 and in2 are the pins used
    public:
        int get(int in1, int in2);        
}
