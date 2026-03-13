#include "tests.h"
#include <Arduino.h>
#include "../ir/IRSensor.h"

void testIR(MotorDriver&, Compass&, I2CManager&) {
    Serial.println("=== IR Sensor Test (DMA) ===");
    Serial.println("Move IR ball around the robot.");
    Serial.println();

    const volatile uint16_t* buf1 = ir_get_buffer(1);
    const volatile uint16_t* buf2 = ir_get_buffer(2);
    const uint32_t count1 = ir_get_sensor_count(1);
    const uint32_t count2 = ir_get_sensor_count(2);

    while (true) {
        if (IR_BOARD1_ENABLED) {
            for (uint32_t s = 0; s < IR_SWEEPS_PER_CYCLE; s++) {
                Serial.print("B1 S");
                Serial.print(s);
                Serial.print(": ");
                for (uint32_t i = 0; i < count1; i++) {
                    if (i > 0) Serial.print('\t');
                    Serial.print(buf1[s * IR_MUX_CHANNELS + i]);
                }
                Serial.println();
            }
        }

        if (IR_BOARD2_ENABLED) {
            for (uint32_t s = 0; s < IR_SWEEPS_PER_CYCLE; s++) {
                Serial.print("B2 S");
                Serial.print(s);
                Serial.print(": ");
                for (uint32_t i = 0; i < count2; i++) {
                    if (i > 0) Serial.print('\t');
                    Serial.print(buf2[s * IR_MUX_CHANNELS + i]);
                }
                Serial.println();
            }
        }
        Serial.println();

        delay(500);
    }
}
