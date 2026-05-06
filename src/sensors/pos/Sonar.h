#ifndef BUCKY_SONAR_H
#define BUCKY_SONAR_H

#include <Arduino.h>
#include "io/gpio/gpio.h"

static constexpr int SONAR_COUNT = 4;
static constexpr uint32_t SONAR_TIMEOUT_US = 20000;

struct SonarPins {
    int trigPin = -1;
    int echoPins[SONAR_COUNT] = {-1, -1, -1, -1};
};

struct SonarReading {
    float distance[SONAR_COUNT] = {};
    bool  valid[SONAR_COUNT] = {};
};

class Sonar {
        GpioPin trigGpio = {nullptr, 0};

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
