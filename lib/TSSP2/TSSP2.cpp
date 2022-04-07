#include <Arduino.h>
#include <TSSP2.h>


TSSP2::TSSP2(int pins[14]) {
    for (int i =0; i < TSSP2::ir_num; i++) {
        TSSP2::pins[i] = pins[i];
    }
}

void TSSP2::setAllSensorPinsInput() {
    for (int i = 0; i < 14; i++) {
        pinMode(pins[i], INPUT);
    }
}

bool TSSP2::getSensorPin(int pin) {
    return digitalRead(pins[pin]);
}

void TSSP2::getAllSensorPulseWidth(float pulseWidth[14], int timeLimit) {
    for (int i = 0; i < ir_num; i++) {
        pulseWidth[i] = 0;
    }

    const long startTime_us = micros();
    do {
        for (int i = 0; i < ir_num; i++) {
            if (getSensorPin(i) == false) {
                pulseWidth[i] += deltaPulseWidth;
            }
        }
    } while ((micros() - startTime_us) < timeLimit);

    maxPulseWidth = 0;
    maxSensorNumber = 0;
    for (int i = 0; i < ir_num; i++) {
        if (pulseWidth[i] > 0) {
            activeSensors += 1;
        }
        if (pulseWidth[i] > maxPulseWidth) {
            maxPulseWidth = pulseWidth[i];
            maxSensorNumber = i;
        }
    }
}


