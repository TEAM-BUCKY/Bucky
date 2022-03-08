// define
#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

// include
#include <Arduino.h>
#include <Core.h>


class Ultrasonic {
    private:
        int trigger, echo; // trigger and echo pins are used
    public:
        Ultrasonic(int trigger, int echo);
        void ultraSetup();
        int read();        
};

#endif