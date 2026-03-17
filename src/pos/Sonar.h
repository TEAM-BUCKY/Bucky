#ifndef BUCKY_SONAR_H
#define BUCKY_SONAR_H

#include <Arduino.h>
#include "io/gpio/gpio.h"

static constexpr int SONAR_COUNT = 4;
static constexpr uint32_t SONAR_TIMEOUT_US = 30000;

struct SonarPins {
    int trigPin = -1;
    int echoPins[SONAR_COUNT] = {-1, -1, -1, -1};
};

struct SonarReading {
    float distance[SONAR_COUNT] = {};
};

class Sonar {
        GpioPin trigGpio = {NULL, 0};

        uint32_t trigStart = 0;
        bool reading = false;

    public:
        void begin(const SonarPins& pins);

        void startRead();
        bool isReadComplete() const;
        SonarReading processRead();

        SonarReading read();
};

#endif //BUCKY_SONAR_H
