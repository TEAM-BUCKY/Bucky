#include "tests.h"
#include <Arduino.h>

void testIR(MotorDriver&, Compass&, IRController& irController, I2CManager&) {
    Serial.println("=== IR Sensor Test ===");
    Serial.print("ADC1: ");
    Serial.println(irController.isAdc1Ok() ? "OK" : "FAIL");
    Serial.print("ADC2: ");
    Serial.println(irController.isAdc2Ok() ? "OK" : "FAIL");

    Serial.println("Calibrating (keep IR ball away)...");
    irController.calibrate(32);

    Serial.println("Baseline values:");
    for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
        if (i > 0) Serial.print('\t');
        Serial.print(irController.getRawValue(i));
    }
    Serial.println();
    Serial.println("Calibration done. Move IR ball around the robot.");
    Serial.println();

    while (true) {
        irController.update();

        IRVector vec = irController.getVector();
        uint8_t strongest = irController.getStrongestSensor();

        // Print baseline-subtracted values
        for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
            if (i > 0) Serial.print('\t');
            Serial.print(irController.getValue(i));
        }

        Serial.print(" | angle: ");
        Serial.print(vec.angle, 1);
        Serial.print(" | str: ");
        Serial.print(vec.strength, 0);
        Serial.print(" | max: S");
        Serial.println(strongest);

        delay(1000);
    }
}
