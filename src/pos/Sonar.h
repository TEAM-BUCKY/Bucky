#ifndef BUCKY_SONAR_H
#define BUCKY_SONAR_H

#include <Arduino.h>

struct SonarPins {
    int trigPin = -1;
    int echoPin = -1;
};

struct SonarReading
{
    double sonar1 = 0;
    double sonar2 = 0;
    double sonar3 = 0;
    double sonar4 = 0;
};

class Sonar
{
    int trigPin;
    int echoPin;

    public:
        explicit Sonar(const SonarPins pins) : trigPin(pins.trigPin), echoPin(pins.echoPin) {}
        void init() const;

        [[nodiscard]] double getDistance() const;

        void changePins(SonarPins pins);
};



#endif //BUCKY_SONAR_H