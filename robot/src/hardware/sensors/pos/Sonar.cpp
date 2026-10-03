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
    const auto idx = static_cast<int>(reinterpret_cast<intptr_t>(ctx));
    if (gpio_read(echoGpio[idx]))
        riseTime[idx] = micros();
    else {
        duration[idx] = micros() - riseTime[idx];
        done[idx] = true;
    }
}

// PA10 shares EXTI line 10 with PC10 on the G474 board. Only one port can drive
// that line, so PA10 falls back to TIM1_CH3 input capture. Timer ticks at 1 MHz so CCR3 is
// directly in microseconds, matching micros()-based timestamps used elsewhere.
static void pa10CaptureCallback() {
    const uint32_t captured = TIM1->CCR3;
    if ((GPIOA->IDR & (1u << 10)) != 0) {
        riseTime[0] = captured;
    } else {
        duration[0] = static_cast<uint16_t>(captured - riseTime[0]);
        done[0] = true;
    }
}

static void setupPA10TimerCapture() {
    static HardwareTimer tim1(TIM1);
    tim1.setMode(3, TIMER_INPUT_CAPTURE_BOTHEDGE, PA_10);
    tim1.setPrescaleFactor(tim1.getTimerClkFreq() / 1'000'000u);
    tim1.setOverflow(0xFFFFu, TICK_FORMAT);
    tim1.attachInterrupt(3, pa10CaptureCallback);
    tim1.resume();
}

void Sonar::begin(const SonarPins& pins) {
    trigGpio = gpio_pin_init(pins.trigPin);
    gpio_mode(trigGpio, OUTPUT);
    gpio_low(trigGpio);

    // Claim EXTI lines first so PA10 only takes the timer path when PC10 owns line 10.
    for (int i = 0; i < SONAR_COUNT; i++) {
        echoPinNames[i] = pins.echoPins[i];
        if (echoPinNames[i] == NC) continue;

        echoGpio[i] = gpio_pin_init(echoPinNames[i]);
        gpio_mode(echoGpio[i], INPUT);

        if (echoPinNames[i] != PA_10)
            attachEcho(echoPinNames[i], i);
    }

    for (int i = 0; i < SONAR_COUNT; i++) {
        if (echoPinNames[i] != PA_10) continue;
        if (!attachEcho(PA_10, i))
            setupPA10TimerCapture();
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
