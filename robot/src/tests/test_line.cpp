#include "tests.h"
#include "debug.h"
#include <Arduino.h>

#include "hardware/sensors/Button.h"
#include "hardware/sensors/LineSensor.h"

static void printRaw(const LineSensor& line) {
    DBG_PRINTLN("sensor\tR\tG\tB\tDark\tR-D\tG-D\tB-D");
    for (uint8_t s = 0; s < LineSensor::SENSORS; s++) {
        DBG_PRINT(s);
        for (const LineColor c : {LineColor::Red, LineColor::Green, LineColor::Blue, LineColor::Dark}) {
            DBG_PRINT('\t'); DBG_PRINT(line.raw().get(c, s));
        }
        for (const LineColor c : {LineColor::Red, LineColor::Green, LineColor::Blue}) {
            DBG_PRINT('\t'); DBG_PRINT(line.reflected(c, s));
        }
        DBG_PRINTLN();
    }
}

static void printWhite(const LineSensor& line) {
    DBG_PRINTLN("sensor\twR\twG\twB");
    for (uint8_t s = 0; s < LineSensor::SENSORS; s++) {
        DBG_PRINT(s);
        for (const LineColor c : {LineColor::Red, LineColor::Green, LineColor::Blue}) {
            DBG_PRINT('\t'); DBG_PRINT(line.whiteLevel(c, s));
        }
        DBG_PRINTLN();
    }
}

static void printBalanced(const LineSensor& line) {
    DBG_PRINTLN("sensor\tR\tG\tB\t(0-255, white = 255)");
    for (uint8_t s = 0; s < LineSensor::SENSORS; s++) {
        DBG_PRINT(s);
        if (!line.hasSensor(s)) {
            DBG_PRINTLN("\t-\t-\t-");
            continue;
        }
        const LineRGB c = line.color(s);
        DBG_PRINT('\t'); DBG_PRINT(c.r);
        DBG_PRINT('\t'); DBG_PRINT(c.g);
        DBG_PRINT('\t'); DBG_PRINTLN(c.b);
    }
}

void testLine(const TestContext& ctx) {
    DBG_PRINTLN("=== Line Sensor Test (G port) ===");
    DBG_PRINTLN("Button 1: calibrate white (hold the sensors over white).");
    DBG_PRINTLN("Button 2: rainbow show on the sensor LEDs.");
    DBG_PRINTLN();

    if (ctx.linePort == nullptr) {
        DBG_PRINTLN("No G port is configured for the line sensors.");
        while (true) {}
    }

    LineSensor line(*ctx.linePort);
    uint32_t lastFrameMs = millis();
    uint32_t lastPrintMs = 0;

    while (true) {

        if (button1.pressed()) {
            DBG_PRINTLN("Calibrating white...");
            if (line.calibrateWhite()) printWhite(line);
            else                       DBG_PRINTLN("Calibration failed: no frames.");
            DBG_PRINTLN();
        }

        if (button2.pressed()) {
            DBG_PRINTLN("Celebrating!");
            if (!line.celebrate()) DBG_PRINTLN("Show failed: the board never sent a reset.");
            DBG_PRINTLN();
        }

        if (!line.update()) {
            if (millis() - lastFrameMs > 1000) {
                lastFrameMs = millis();
                DBG_PRINT("no frame: seq=");  DBG_PRINT(ctx.linePort->frameSequence());
                DBG_PRINT(" desyncs=");       DBG_PRINTLN(ctx.linePort->desyncCount());
            }
            delay(1);
            continue;
        }
        lastFrameMs = millis();

        if (millis() - lastPrintMs < 500) continue;
        lastPrintMs = millis();

        DBG_PRINT("seq=");      DBG_PRINT(ctx.linePort->frameSequence());
        DBG_PRINT(" desyncs="); DBG_PRINTLN(ctx.linePort->desyncCount());

        if (line.calibrated()) printBalanced(line);
        else                   printRaw(line);
        DBG_PRINTLN();
    }
}
