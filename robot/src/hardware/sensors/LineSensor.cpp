#include "hardware/sensors/LineSensor.h"

static constexpr LineColor RGB[3] = {LineColor::Red, LineColor::Green, LineColor::Blue};

bool LineSensor::update() {
    if (!port.hasNewFrame(lastSeq) || !port.readLine(frame)) return false;
    lastSeq = port.frameSequence();
    return true;
}

bool LineSensor::waitFrame(const uint32_t timeoutMs) {
    const uint32_t start = millis();
    while (!update()) {
        if (millis() - start > timeoutMs) return false;
        delay(1);
    }
    return true;
}

int32_t LineSensor::reflected(const LineColor color, const uint8_t sensor) const {
    return static_cast<int32_t>(frame.get(color, sensor))
         - static_cast<int32_t>(frame.get(LineColor::Dark, sensor));
}

bool LineSensor::calibrateWhite(const uint8_t frames) {
    if (frames == 0) return false;

    int32_t sum[3][SENSORS] = {};
    for (uint8_t n = 0; n < frames; n++) {
        if (!waitFrame(500)) return false;
        for (uint8_t c = 0; c < 3; c++)
            for (uint8_t s = 0; s < SENSORS; s++)
                sum[c][s] += reflected(RGB[c], s);
    }

    for (uint8_t c = 0; c < 3; c++)
        for (uint8_t s = 0; s < SENSORS; s++)
            white[c][s] = sum[c][s] / frames;
    isCalibrated = true;
    return true;
}

int32_t LineSensor::whiteLevel(const LineColor color, const uint8_t sensor) const {
    if (color == LineColor::Dark) return 0;
    return white[static_cast<uint8_t>(color)][sensor];
}

bool LineSensor::hasSensor(const uint8_t sensor) const {
    if (!isCalibrated) return false;
    for (uint8_t c = 0; c < 3; c++)
        if (white[c][sensor] < MIN_WHITE_SPAN) return false;
    return true;
}

LineRGB LineSensor::color(const uint8_t sensor) const {
    if (!hasSensor(sensor)) return {0, 0, 0};

    uint8_t out[3];
    for (uint8_t c = 0; c < 3; c++) {
        const int32_t v = reflected(RGB[c], sensor) * 255 / white[c][sensor];
        out[c] = static_cast<uint8_t>(constrain(v, 0, 255));
    }
    return {out[0], out[1], out[2]};
}

// ---- Show mode -------------------------------------------------------------

// Takes the clock and steps until the board's reset fires, which puts its
// counter at the start of the first colour sweep.
bool LineSensor::beginShow() {
    if (port.kind() != GSensorKind::Line || !port.holdClock()) return false;

    const uint32_t resets = port.resetCount();
    for (uint32_t i = 0; i < 2 * GPort::MAX_FRAME; i++) {
        port.stepClock();
        if (port.resetCount() != resets) return true;
    }

    endShow();   // no reset seen: board missing or not clocking
    return false;
}

void LineSensor::endShow() {
    port.releaseClock();
    lastSeq = port.frameSequence();
}

// One full frame (4 sweeps x 16 steps), starting and ending at the first
// sweep. Each colour sweep lasts in proportion to its level; Dark takes the rest.
void LineSensor::showPeriod(const LineRGB color) {
    const uint8_t level[4] = {color.r, color.g, color.b, 0};   // indexed by LineColor

    uint32_t dwell[4];
    uint32_t lit = 0;
    for (uint8_t sweep = 0; sweep < 4; sweep++) {
        const auto c = static_cast<uint8_t>(port.lineColorAt(sweep));
        dwell[sweep] = SHOW_PERIOD_US / 3 * level[c] / 255;
        lit += dwell[sweep];
    }
    for (uint8_t sweep = 0; sweep < 4; sweep++)
        if (port.lineColorAt(sweep) == LineColor::Dark) dwell[sweep] = SHOW_PERIOD_US - lit;

    uint32_t deadline = micros();
    for (const uint32_t d : dwell) {
        for (uint32_t ch = 0; ch < GPort::SENSORS; ch++) {
            deadline += d / GPort::SENSORS;
            while (static_cast<int32_t>(micros() - deadline) < 0) {}
            port.stepClock();
        }
    }
}

bool LineSensor::showColor(const LineRGB color, const uint32_t durationMs) {
    if (!beginShow()) return false;

    const uint32_t start = millis();
    while (millis() - start < durationMs) showPeriod(color);

    endShow();
    return true;
}

static LineRGB hueToRgb(const uint32_t hue) {   // hue 0..359, full saturation/value
    const uint32_t h = hue % 360;
    const auto rise = static_cast<uint8_t>((h % 60) * 255 / 60);
    const auto fall = static_cast<uint8_t>(255 - rise);
    switch (h / 60) {
        case 0:  return {255, rise, 0};
        case 1:  return {fall, 255, 0};
        case 2:  return {0, 255, rise};
        case 3:  return {0, fall, 255};
        case 4:  return {rise, 0, 255};
        default: return {255, 0, fall};
    }
}

bool LineSensor::celebrate(const uint32_t durationMs) {
    if (!beginShow()) return false;

    const uint32_t start = millis();
    uint32_t elapsed;
    while ((elapsed = millis() - start) < durationMs)
        showPeriod(hueToRgb(elapsed * 360 / 1000));   // one rainbow per second

    endShow();
    return true;
}
