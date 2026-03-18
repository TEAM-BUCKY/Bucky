#include "Sonar.h"
#include "io/gpio/gpio.h"

static GpioPin echoGpio[SONAR_COUNT];
static int echoPinNumbers[SONAR_COUNT];

static volatile uint32_t riseTime[SONAR_COUNT];
static volatile uint32_t duration[SONAR_COUNT];
static volatile bool done[SONAR_COUNT];

static void echoISR(const int idx) {
    if (gpio_read(echoGpio[idx]))
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

void Sonar::begin(const SonarPins& pins) {
    trigGpio = gpio_pin_init(pins.trigPin);
    gpio_mode(trigGpio, OUTPUT);
    gpio_low(trigGpio);

    for (int i = 0; i < SONAR_COUNT; i++) {
        echoPinNumbers[i] = pins.echoPins[i];
        if (echoPinNumbers[i] >= 0) {
            echoGpio[i] = gpio_pin_init(echoPinNumbers[i]);
            gpio_mode(echoGpio[i], INPUT);
            attachInterrupt(digitalPinToInterrupt(echoPinNumbers[i]), isrTable[i], CHANGE);
        }
    }
}

void Sonar::startRead() {
    for (int i = 0; i < SONAR_COUNT; i++) {
        riseTime[i] = 0;
        duration[i] = 0;
        done[i] = false;
    }

    gpio_low(trigGpio);
    delayMicroseconds(2);
    gpio_high(trigGpio);
    delayMicroseconds(10);
    gpio_low(trigGpio);

    trigStart = micros();
    reading = true;
}

bool Sonar::isReadComplete() const {
    if (!reading) return true;

    if (micros() - trigStart >= SONAR_TIMEOUT_US) return true;

    for (int i = 0; i < SONAR_COUNT; i++)
        if (echoPinNumbers[i] >= 0 && !done[i])
            return false;

    return true;
}

SonarReading Sonar::processRead() {
    reading = false;

    SonarReading r;
    for (int i = 0; i < SONAR_COUNT; i++)
        r.distance[i] = 0.017f * duration[i];

    return r;
}

SonarReading Sonar::read() {
    startRead();

    while (!isReadComplete()) {}

    return processRead();
}
