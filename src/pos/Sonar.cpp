#include "Sonar.h"
#include "../gpio/gpio.h"

static GpioPin trigGpio{nullptr, 0};
static GpioPin echoGpio[SONAR_COUNT] = {{nullptr, 0}, {nullptr, 0}, {nullptr, 0}, {nullptr, 0}};
static int echoPinNumbers[SONAR_COUNT];

static volatile uint32_t riseTime[SONAR_COUNT];
static volatile uint32_t duration[SONAR_COUNT];
static volatile bool done[SONAR_COUNT];

static void echoISR(const int idx) {
    if (gpioRead(echoGpio[idx]))
        riseTime[idx] = micros();
    else {
        duration[idx] = micros() - riseTime[idx];
        done[idx] = true;
    }
}

static void echoISR0() { echoISR(0); }
static void echoISR1() { echoISR(1); }
static void echoISR2() { echoISR(2); }
static void echoISR3() { echoISR(3); }

static constexpr void (*const isrTable[SONAR_COUNT])() = {echoISR0, echoISR1, echoISR2, echoISR3};

void setupSonar(const SonarPins& pins) {
    trigGpio = GpioPin(pins.trigPin);
    gpioMode(trigGpio, OUTPUT);
    gpioLow(trigGpio);

    for (int i = 0; i < SONAR_COUNT; i++) {
        echoPinNumbers[i] = pins.echoPins[i];
        if (echoPinNumbers[i] >= 0) {
            echoGpio[i] = GpioPin(echoPinNumbers[i]);
            gpioMode(echoGpio[i], INPUT);
        }
    }
}

SonarReading readSonars() {
    for (int i = 0; i < SONAR_COUNT; i++) {
        riseTime[i] = 0;
        duration[i] = 0;
        done[i] = false;
    }

    for (int i = 0; i < SONAR_COUNT; i++)
        attachInterrupt(digitalPinToInterrupt(echoPinNumbers[i]), isrTable[i], CHANGE);

    gpioLow(trigGpio);
    delayMicroseconds(2);
    gpioHigh(trigGpio);
    delayMicroseconds(10);
    gpioLow(trigGpio);

    const uint32_t start = micros();
    bool allDone = false;
    while (!allDone && micros() - start < SONAR_TIMEOUT_US) {
        allDone = true;
        for (int i = 0; i < SONAR_COUNT; i++) {
            if (echoPinNumbers[i] >= 0 && !done[i]) {
                allDone = false;
            }
        }
    }

    for (const auto pin : echoPinNumbers)
        detachInterrupt(digitalPinToInterrupt(pin));

    SonarReading reading;
    for (int i = 0; i < SONAR_COUNT; i++)
        reading.distance[i] = 0.034 * duration[i] / 2.0;

    return reading;
}
