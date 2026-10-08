#ifndef BUCKY_LINESENSOR_H
#define BUCKY_LINESENSOR_H

#include <Arduino.h>

#include "hardware/sensors/GPort.h"

struct LineRGB {
    uint8_t r, g, b;
};

// Line sensor ring on a G port: white-balanced colour per sensor, and a show
// mode that drives the sensor LEDs as an RGB light.
class LineSensor {
public:
    static constexpr uint8_t SENSORS = GPort::SENSORS;

    explicit LineSensor(GPort& port) : port(port) {}

    // Pulls the newest frame from the port. Returns true when it was new.
    bool update();

    [[nodiscard]] const LineFrame& raw() const { return frame; }

    // Light reflected back for one colour, ambient (Dark) subtracted.
    [[nodiscard]] int32_t reflected(LineColor color, uint8_t sensor) const;

    // Blocking: averages `frames` frames over a white surface as the 255 reference.
    // Returns false (keeping the old reference) when frames stop arriving.
    bool calibrateWhite(uint8_t frames = 32);

    [[nodiscard]] bool calibrated() const { return isCalibrated; }
    [[nodiscard]] int32_t whiteLevel(LineColor color, uint8_t sensor) const;

    // False when a channel had too little signal over white (unplugged or dead).
    [[nodiscard]] bool hasSensor(uint8_t sensor) const;

    // White-balanced colour, 0..255 per channel (white = 255, 255, 255).
    [[nodiscard]] LineRGB color(uint8_t sensor) const;

    // Show mode (blocking). The line board picks the LED colour from where its
    // counter is in the frame, so the mux clock is stepped by hand with a dwell
    // per colour sweep: the time spent in each sweep mixes the colour.
    // Sensor readings stop while a show runs and resume afterwards.
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
    void showPeriod(LineRGB color);
    bool waitFrame(uint32_t timeoutMs);
};

#endif // BUCKY_LINESENSOR_H
