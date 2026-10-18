#ifndef BUCKY_LINESENSOR_H
#define BUCKY_LINESENSOR_H

#include <Arduino.h>

#include <hardware/sensors/gport/GPort.h>

struct LineRGB {
    uint8_t r, g, b;
};

class LineSensor {
public:
    static constexpr uint8_t SENSORS = GPort::SENSORS;

    explicit LineSensor(GPort& port) : port(port) {}

    bool update();

    [[nodiscard]] const LineFrame& raw() const { return frame; }

    [[nodiscard]] int32_t reflected(LineColor color, uint8_t sensor) const;

    bool calibrateWhite(uint8_t frames = 32);

    [[nodiscard]] bool calibrated() const { return isCalibrated; }
    [[nodiscard]] int32_t whiteLevel(LineColor color, uint8_t sensor) const;

    [[nodiscard]] bool hasSensor(uint8_t sensor) const;

    [[nodiscard]] LineRGB color(uint8_t sensor) const;

    bool showColor(LineRGB color, uint32_t durationMs);
    bool celebrate(uint32_t durationMs = 3000);   // rainbow

private:
    static constexpr int32_t MIN_WHITE_SPAN = 20;
    static constexpr uint32_t SHOW_PERIOD_US = 4000;   // one colour mix, 250 Hz

    GPort& port;
    LineFrame frame = {};
    uint32_t lastSeq = 0;

    int32_t white[3][SENSORS] = {};
    bool isCalibrated = false;

    bool beginShow();
    void endShow();
    void showPeriod(LineRGB color) const;
    bool waitFrame(uint32_t timeoutMs);
};

#endif // BUCKY_LINESENSOR_H
