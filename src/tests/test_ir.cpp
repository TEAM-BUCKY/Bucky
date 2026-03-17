#include "tests.h"
#include "debug.h"
#include <Arduino.h>
#include "../ir/IRSensor.h"

void testIR(TestContext&) {
    DBG_PRINTLN("=== IR Sensor Test (DMA) ===");
    DBG_PRINTLN("Move IR ball around the robot.");
    DBG_PRINTLN();

    const volatile uint16_t* buf1 = ir_get_buffer(1);
    const volatile uint16_t* buf2 = ir_get_buffer(2);
    const uint32_t count1 = ir_get_sensor_count(1);
    const uint32_t count2 = ir_get_sensor_count(2);

    while (true) {
        if (IR_BOARD1_ENABLED) {
            for (uint32_t s = 0; s < IR_SWEEPS_PER_CYCLE; s++) {
                DBG_PRINT("B1 S");
                DBG_PRINT(s);
                DBG_PRINT(": ");
                for (uint32_t i = 0; i < count1; i++) {
                    if (i > 0) DBG_PRINT('\t');
                    DBG_PRINT(buf1[s * IR_MUX_CHANNELS + i]);
                }
                DBG_PRINTLN();
            }
        }

        if (IR_BOARD2_ENABLED) {
            for (uint32_t s = 0; s < IR_SWEEPS_PER_CYCLE; s++) {
                DBG_PRINT("B2 S");
                DBG_PRINT(s);
                DBG_PRINT(": ");
                for (uint32_t i = 0; i < count2; i++) {
                    if (i > 0) DBG_PRINT('\t');
                    DBG_PRINT(buf2[s * IR_MUX_CHANNELS + i]);
                }
                DBG_PRINTLN();
            }
        }
        DBG_PRINTLN();

        delay(500);
    }
}
