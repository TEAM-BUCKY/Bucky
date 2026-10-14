#include "IRRing.h"

#include <cmath>

#include "helpers/Math.h"

namespace
{
    constexpr float MIN_COS = 1e-3f;

    struct FitPoint {
        float sinA, cosA;   // sensor axis (x = sin, y = cos)
        float lnAmp;
        float weight;
    };

    // ln of the modelled amplitude at one sensor for a ball at (bearing, centre distance).
    float modelLnAmp(const FitPoint& p, const float bearing, const float rho, const IRConfig& cfg)
    {
        const float bx = rho * sinf(bearing), by = rho * cosf(bearing);
        const float dx = bx - cfg.mountRadiusCm * p.sinA, dy = by - cfg.mountRadiusCm * p.cosA;
        const float d = sqrtf(dx * dx + dy * dy);
        float c = (dx * p.sinA + dy * p.cosA) / d;
        if (c < MIN_COS) c = MIN_COS;
        return 2.0f * logf(cfg.kCm) - 2.0f * logf(d) + cfg.cosPower * logf(c);
    }

    // Offset δ ∈ [0, step/2] of the ball from the peak sensor towards its stronger neighbour,
    // from the far-field ratio ln(A_peak / A_neighbour) = n·(ln cos δ − ln cos(step − δ)).
    float farFieldOffset(const float ratioLn, const float step, const float n)
    {
        const float rho = ratioLn / n;
        if (rho <= 0.0f) return step * 0.5f;
        if (rho >= -logf(cosf(step))) return 0.0f;
        float d = step * 0.5f - rho / step;
        for (int i = 0; i < 4; i++) {
            const float f = logf(cosf(d)) - logf(cosf(step - d)) - rho;
            const float df = -tanf(d) - tanf(step - d);
            d -= f / df;
            if (d < 0.0f) d = 0.0f;
            if (d > step * 0.5f) d = step * 0.5f;
        }
        return d;
    }
}

float IRRing::sensorAngle(const uint8_t sensor, const IRConfig& cfg)
{
    return 2.0f * _PI_F * static_cast<float>(sensor) / static_cast<float>(cfg.sensors);
}

float IRRing::sensorDistanceCm(const float amp, const IRConfig& cfg)
{
    if (amp <= 0.0f) return cfg.maxCm;
    const float r = cfg.kCm / sqrtf(amp);
    return r > cfg.maxCm ? cfg.maxCm : r;
}

BallReading IRRing::estimate(const uint16_t raw[GPort::SENSORS], const IRConfig& cfg)
{
    BallReading out;
    const uint8_t n = cfg.sensors > GPort::SENSORS ? GPort::SENSORS : cfg.sensors;
    if (n < 3) return out;
    const float step = 2.0f * _PI_F / static_cast<float>(n);

    float amp[GPort::SENSORS] = {};
    bool clip[GPort::SENSORS] = {};
    uint8_t peak = 0;
    for (uint8_t k = 0; k < n; k++) {
        const float a = static_cast<float>(raw[k]) - cfg.baselineCode;
        amp[k] = a > 0.0f ? a : 0.0f;
        clip[k] = raw[k] >= cfg.clipCode;
        if (amp[k] > amp[peak]) peak = k;
    }
    out.peakSensor = peak;
    out.peakAmp = amp[peak];
    if (amp[peak] < cfg.minAmp) return out;

    out.seen = true;
    out.clipped = clip[peak];
    const uint8_t left = (peak + n - 1) % n, right = (peak + 1) % n;

    // Far-field start: bearing from neighbour ratios, range from the peak.
    float offset = 0.0f;
    if (!clip[peak]) {
        const bool rightStronger = amp[right] >= amp[left];
        const uint8_t j = rightStronger ? right : left;
        if (amp[j] >= cfg.minNeighbourAmp) {
            const float d = farFieldOffset(logf(amp[peak] / amp[j]), step, cfg.cosPower);
            offset = rightStronger ? d : -d;
        }
    } else if (amp[left] >= cfg.minNeighbourAmp && amp[right] >= cfg.minNeighbourAmp) {
        // Peak saturated: balance the two neighbours, ln(A_r/A_l) = n·(ln cos(step−δ) − ln cos(step+δ)).
        const float target = logf(amp[right] / amp[left]) / cfg.cosPower;
        float d = 0.0f;
        for (int i = 0; i < 6; i++) {
            const float f = logf(cosf(step - d)) - logf(cosf(step + d)) - target;
            const float df = tanf(step - d) + tanf(step + d);
            d -= f / df;
            if (d > step * 0.5f) d = step * 0.5f;
            if (d < -step * 0.5f) d = -step * 0.5f;
        }
        offset = d;
    }
    const float peakAngle = sensorAngle(peak, cfg);
    float bearing = peakAngle + offset;
    const float cosOff = cosf(offset);
    const float onAxisAmp = amp[peak] / powf(cosOff > MIN_COS ? cosOff : MIN_COS, cfg.cosPower);
    float rho = cfg.mountRadiusCm + cfg.kCm / sqrtf(onAxisAmp);

    // Near-field refinement: weighted Gauss–Newton on (bearing, centre distance) over the peak
    // and its neighbours, using each sensor's real position on the ring (parallax).
    FitPoint pts[3];
    uint8_t used = 0;
    const uint8_t cand[3] = {left, peak, right};
    for (const uint8_t k : cand) {
        if (clip[k] || amp[k] < cfg.minNeighbourAmp) continue;
        const float a = sensorAngle(k, cfg);
        pts[used++] = {sinf(a), cosf(a), logf(amp[k]), amp[k] * amp[k]};
    }
    out.contributors = used;
    if (used >= 2) {
        const float minRho = cfg.mountRadiusCm + 2.0f;
        for (int iter = 0; iter < 4; iter++) {
            float jtj00 = 0, jtj01 = 0, jtj11 = 0, jtr0 = 0, jtr1 = 0;
            const float hb = 1e-3f, hr = 1e-2f * rho;
            for (uint8_t i = 0; i < used; i++) {
                const float m = modelLnAmp(pts[i], bearing, rho, cfg);
                const float jb = (modelLnAmp(pts[i], bearing + hb, rho, cfg) - m) / hb;
                const float jr = (modelLnAmp(pts[i], bearing, rho + hr, cfg) - m) / hr;
                const float r = pts[i].lnAmp - m;
                const float w = pts[i].weight;
                jtj00 += w * jb * jb; jtj01 += w * jb * jr; jtj11 += w * jr * jr;
                jtr0 += w * jb * r; jtr1 += w * jr * r;
            }
            const float det = jtj00 * jtj11 - jtj01 * jtj01;
            if (fabsf(det) < 1e-12f) break;
            bearing += (jtj11 * jtr0 - jtj01 * jtr1) / det;
            rho += (jtj00 * jtr1 - jtj01 * jtr0) / det;
            if (bearing > peakAngle + step) bearing = peakAngle + step;
            if (bearing < peakAngle - step) bearing = peakAngle - step;
            if (rho < minRho) rho = minRho;
        }
    }

    if (out.clipped) {
        const float clipRho = cfg.mountRadiusCm + cfg.kCm / sqrtf(cfg.clipCode - cfg.baselineCode);
        if (rho > clipRho) rho = clipRho;
    }
    if (rho > cfg.maxCm) rho = cfg.maxCm;

    out.angle = Math::wrapRadians(bearing);
    out.distanceCm = rho;
    const float bx = rho * sinf(out.angle) - cfg.mountRadiusCm * sinf(peakAngle);
    const float by = rho * cosf(out.angle) - cfg.mountRadiusCm * cosf(peakAngle);
    out.sensorDistanceCm = sqrtf(bx * bx + by * by);
    out.position = {rho * sinf(out.angle), rho * cosf(out.angle)};
    float conf = (amp[peak] - cfg.minAmp) / (4.0f * cfg.minAmp);
    out.confidence = conf < 0.0f ? 0.0f : (conf > 1.0f ? 1.0f : conf);
    return out;
}

bool IRRing::update()
{
    if (!ready() || !port.hasNewFrame(lastSeq)) return false;
    const uint32_t seq = port.frameSequence();
    if (!port.readIR(frame)) return false;
    lastSeq = seq;
    last = estimate(frame, cfg);
    return true;
}
