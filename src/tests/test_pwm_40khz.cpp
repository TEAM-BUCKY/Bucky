#include "tests.h"
#include "debug.h"
#include "hal/system.h"
#include "io/gpio/pwm.h"

void testPwm40kHz(const TestContext& ctx) {
    DBG_PRINTLN("=== 40 kHz Square Wave on PB1 ===");

    PwmPinDef pinDef = {GPIOB, 1, 2, TIM3, 3, 0}; // PB1: TIM3_CH4, AF2
    PwmPin pin = pwm_pin_init(&pinDef);
    pwm_init(&pin, &pinDef, 40000, 1);              // 40 kHz, ARR=1
    pwm_write(&pin, 1);                              // 50% duty (square wave)

    DBG_PRINTLN("PB1 running at 40 kHz");

    while (true) {
        delay(1000);
    }
}
