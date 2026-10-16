#include "Sonar.h"
#include "hardware/io/gpio/gpio.h"
#include <HardwareTimer.h>

#include "hardware/io/irq/IRQ.h"

#include "debug.h"

static GpioPin echoGpio[SONAR_COUNT];
static PinName echoPinNames[SONAR_COUNT];

static volatile uint32_t riseTime[SONAR_COUNT];
static volatile uint32_t duration[SONAR_COUNT];
static volatile bool done[SONAR_COUNT];

constexpr uint8_t SONAR_EXTI_PRIORITY = 2;

static void echoISR(void* ctx);

static bool attachEcho(const PinName pin, const int idx) {
    return exti_attach(pin, EXTI_BOTH, echoISR, reinterpret_cast<void*>(static_cast<intptr_t>(idx)),
                       SONAR_EXTI_PRIORITY);
}

static void echoISR(void* ctx) {
    const auto idx = reinterpret_cast<intptr_t>(ctx);
    if (gpio_read(echoGpio[idx]))
        riseTime[idx] = micros();
    else {
        duration[idx] = micros() - riseTime[idx];
        done[idx] = true;
    }
}

void Sonar::begin(const SonarPins& pins) {
    trigGpio = gpio_pin_init(pins.trigPin);
    gpio_mode(trigGpio, OUTPUT);
    gpio_low(trigGpio);

    for (int i = 0; i < SONAR_COUNT; i++) {
        echoPinNames[i] = pins.echoPins[i];
        if (echoPinNames[i] == NC) continue;

        echoGpio[i] = gpio_pin_init(echoPinNames[i]);
        gpio_mode(echoGpio[i], INPUT);
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
        if (echoPinNames[i] != NC && !done[i])
            return false;

    return true;
}

SonarReading Sonar::processRead() {
    reading = false;
    DBG_PRINT_SUBJECT(DEBUG_SUBJ_POSITION, "Processing sonar read: ");
    DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_POSITION, micros());

    SonarReading r;
    for (int i = 0; i < SONAR_COUNT; i++) {
        r.valid[i] = (echoPinNames[i] != NC) && done[i];
        DBG_PRINT_SUBJECT(DEBUG_SUBJ_POSITION, "Sonar ");
        DBG_PRINT_SUBJECT(DEBUG_SUBJ_POSITION, i);
        DBG_PRINT_SUBJECT(DEBUG_SUBJ_POSITION, " distance: ");
        DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_POSITION, r.distance[i]);
        r.distance[i] = r.valid[i] ? 0.017f * duration[i] : 0.0f;
    }

    return r;
}

SonarReading Sonar::read() {
    startRead();

    while (!isReadComplete()) {}

    return processRead();
}
