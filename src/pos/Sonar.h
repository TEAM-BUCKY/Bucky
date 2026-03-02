#ifndef BUCKY_SONAR_H
#define BUCKY_SONAR_H

#include <Arduino.h>

static constexpr int SONAR_COUNT = 4;
static constexpr uint32_t SONAR_TIMEOUT_US = 30000; // ~5m max range

struct SonarPins {
    int trigPin = -1;
    int echoPins[SONAR_COUNT] = {-1, -1, -1, -1};
};

struct SonarReading {
    double distance[SONAR_COUNT] = {};
};

void setupSonar(const SonarPins& pins);
SonarReading readSonars();

#endif //BUCKY_SONAR_H
