#include "IRController.h"
#include <cmath>

constexpr float IRController::unitX[IR_SENSOR_COUNT];
constexpr float IRController::unitY[IR_SENSOR_COUNT];

IRController::IRController() : adc1(IR_ADC1_ADDR), adc2(IR_ADC2_ADDR) {}

bool IRController::init(TwoWire& wire) {
    adc1Ok = adc1.init(wire);
    adc2Ok = adc2.init(wire);

    Serial.print("IR ADC1 (0x");
    Serial.print(IR_ADC1_ADDR, HEX);
    Serial.print("): ");
    Serial.println(adc1Ok ? "OK" : "FAIL");

    Serial.print("IR ADC2 (0x");
    Serial.print(IR_ADC2_ADDR, HEX);
    Serial.print("): ");
    Serial.println(adc2Ok ? "OK" : "FAIL");

    return adc1Ok || adc2Ok;
}

void IRController::calibrate(uint8_t samples) {
    uint32_t accum[IR_SENSOR_COUNT] = {};

    for (uint8_t s = 0; s < samples; s++) {
        if (adc1Ok) adc1.readAllChannels(rawValues);
        if (adc2Ok) adc2.readAllChannels(rawValues + TLA2528_NUM_CHANNELS);
        for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
            accum[i] += rawValues[i];
        }
        delay(10);
    }

    for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
        baseline[i] = accum[i] / samples;
    }
}

void IRController::update() {
    if (adc1Ok) adc1.readAllChannels(rawValues);
    if (adc2Ok) adc2.readAllChannels(rawValues + TLA2528_NUM_CHANNELS);
}

IRVector IRController::getVector() const {
    float sumX = 0;
    float sumY = 0;

    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        int16_t val = (int16_t)rawValues[i] - (int16_t)baseline[i];
        if (val <= 0) continue;
        sumX += val * unitX[i];
        sumY += val * unitY[i];
    }

    float strength = sqrtf(sumX * sumX + sumY * sumY);
    if (strength < 1.0f) {
        return {0.0f, 0.0f};
    }

    float angle = atan2f(sumX, sumY) * 180.0f / PI;
    if (angle < 0) angle += 360.0f;

    return {angle, strength};
}

uint16_t IRController::getRawValue(uint8_t index) const {
    if (index >= IR_SENSOR_COUNT) return 0;
    return rawValues[index];
}

int16_t IRController::getValue(uint8_t index) const {
    if (index >= IR_SENSOR_COUNT) return 0;
    int16_t val = (int16_t)rawValues[index] - (int16_t)baseline[index];
    return (val > 0) ? val : 0;
}

uint8_t IRController::getStrongestSensor() const {
    uint8_t maxIdx = 0;
    int16_t maxVal = 0;
    for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
        int16_t val = (int16_t)rawValues[i] - (int16_t)baseline[i];
        if (val > maxVal) {
            maxVal = val;
            maxIdx = i;
        }
    }
    return maxIdx;
}
