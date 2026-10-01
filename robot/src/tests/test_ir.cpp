#include "tests.h"
#include "debug.h"
#include <Arduino.h>

void testIR(const TestContext& ctx) {
    DBG_PRINTLN("=== IR Sensor Test (G port) ===");
    DBG_PRINTLN();

    if (ctx.irPort == nullptr) {
        DBG_PRINTLN("No G port is configured for the IR ring.");
        while (true) {}
    }

    uint16_t frame[GPort::SENSORS];
    uint32_t lastSeq = ctx.irPort->frameSequence();

    while (true) {
        if (!ctx.irPort->hasNewFrame(lastSeq) || !ctx.irPort->readIR(frame)) {
            delay(1);
            continue;
        }
        lastSeq = ctx.irPort->frameSequence();

        DBG_PRINT("seq=");      DBG_PRINT(lastSeq);
        DBG_PRINT(" desyncs="); DBG_PRINTLN(ctx.irPort->desyncCount());

        for (uint32_t s = 0; s < GPort::SENSORS; s++) {
            if (s > 0) DBG_PRINT('\t');
            DBG_PRINT(frame[s]);
        }
        DBG_PRINTLN();
        DBG_PRINTLN();

        delay(500);
    }
}
