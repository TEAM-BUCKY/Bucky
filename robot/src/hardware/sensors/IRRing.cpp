#include "IRRing.h"

#include <cmath>

#include "helpers/Math.h"

namespace
{
    constexpr float MIN_COS = 1e-3f;

    struct FitPoint {
        float sinA, cosA;
        float lnAmp;
        float weight;
    };

    // ln of the modelled amplitude at one sensor for a ball at (bearing, centre distance).
    float modelLnAmp(const FitPoint& sensor, const float bearing, const float centreDist, const IRConfig& cfg)
    {
        const float ballX = centreDist * sinf(bearing), ballY = centreDist * cosf(bearing);
        const float toBallX = ballX - cfg.mountRadiusCm * sensor.sinA;
        const float toBallY = ballY - cfg.mountRadiusCm * sensor.cosA;
        const float sensorDist = sqrtf(toBallX * toBallX + toBallY * toBallY);
        float cosIncidence = (toBallX * sensor.sinA + toBallY * sensor.cosA) / sensorDist;
        if (cosIncidence < MIN_COS) cosIncidence = MIN_COS;
        return 2.0f * logf(cfg.kCm) - 2.0f * logf(sensorDist) + cfg.cosPower * logf(cosIncidence);
    }

    // Offset δ ∈ [0, step/2] of the ball from the peak sensor towards its stronger neighbour,
    // from the far-field ratio ln(A_peak / A_neighbour) = n·(ln cos δ − ln cos(step − δ)).
    float farFieldOffset(const float lnPeakToNeighbour, const float sensorStep, const float cosPower)
    {
        const float target = lnPeakToNeighbour / cosPower;
        if (target <= 0.0f) return sensorStep * 0.5f;
        if (target >= -logf(cosf(sensorStep))) return 0.0f;
        float offset = sensorStep * 0.5f - target / sensorStep;
        for (int iter = 0; iter < 4; iter++) {
            const float error = logf(cosf(offset)) - logf(cosf(sensorStep - offset)) - target;
            const float slope = -tanf(offset) - tanf(sensorStep - offset);
            offset -= error / slope;
            if (offset < 0.0f) offset = 0.0f;
            if (offset > sensorStep * 0.5f) offset = sensorStep * 0.5f;
        }
        return offset;
    }
}

float IRRing::sensorAngle(const uint8_t sensor, const IRConfig& cfg)
{
    return 2.0f * _PI_F * static_cast<float>(sensor) / static_cast<float>(cfg.sensors);
}

float IRRing::sensorDistanceCm(const float amp, const IRConfig& cfg)
{
    if (amp <= 0.0f) return cfg.maxCm;
    const float dist = cfg.kCm / sqrtf(amp);
    return dist > cfg.maxCm ? cfg.maxCm : dist;
}

BallReading IRRing::estimate(const uint16_t frame[GPort::SENSORS], const IRConfig& cfg)
{
    BallReading out;
    const uint8_t sensorCount = cfg.sensors > GPort::SENSORS ? GPort::SENSORS : cfg.sensors;
    if (sensorCount < 3) return out;
    const float sensorStep = 2.0f * _PI_F / static_cast<float>(sensorCount);

    float amp[GPort::SENSORS] = {};
    bool clipped[GPort::SENSORS] = {};
    uint8_t peak = 0;
    for (uint8_t s = 0; s < sensorCount; s++) {
        const float aboveBaseline = static_cast<float>(frame[s]) - cfg.baselineCode;
        amp[s] = aboveBaseline > 0.0f ? aboveBaseline : 0.0f;
        clipped[s] = frame[s] >= cfg.clipCode;
        if (amp[s] > amp[peak]) peak = s;
    }
    out.peakSensor = peak;
    out.peakAmp = amp[peak];
    if (amp[peak] < cfg.minAmp) return out;

    out.seen = true;
    out.clipped = clipped[peak];
    const uint8_t left = (peak + sensorCount - 1) % sensorCount, right = (peak + 1) % sensorCount;

    float offset = 0.0f;
    if (!clipped[peak]) {
        const bool rightStronger = amp[right] >= amp[left];
        const uint8_t neighbour = rightStronger ? right : left;
        if (amp[neighbour] >= cfg.minNeighbourAmp) {
            const float delta = farFieldOffset(logf(amp[peak] / amp[neighbour]), sensorStep, cfg.cosPower);
            offset = rightStronger ? delta : -delta;
        }
    } else if (amp[left] >= cfg.minNeighbourAmp && amp[right] >= cfg.minNeighbourAmp) {
        const float target = logf(amp[right] / amp[left]) / cfg.cosPower;
        float delta = 0.0f;
        for (int iter = 0; iter < 6; iter++) {
            const float error = logf(cosf(sensorStep - delta)) - logf(cosf(sensorStep + delta)) - target;
            const float slope = tanf(sensorStep - delta) + tanf(sensorStep + delta);
            delta -= error / slope;
            if (delta > sensorStep * 0.5f) delta = sensorStep * 0.5f;
            if (delta < -sensorStep * 0.5f) delta = -sensorStep * 0.5f;
        }
        offset = delta;
    }
    const float peakAngle = sensorAngle(peak, cfg);
    float bearing = peakAngle + offset;
    const float cosOffset = cosf(offset);
    const float onAxisAmp = amp[peak] / powf(cosOffset > MIN_COS ? cosOffset : MIN_COS, cfg.cosPower);
    float centreDist = cfg.mountRadiusCm + cfg.kCm / sqrtf(onAxisAmp);

    FitPoint fitPoints[3];
    uint8_t fitCount = 0;
    const uint8_t candidates[3] = {left, peak, right};
    for (const uint8_t s : candidates) {
        if (clipped[s] || amp[s] < cfg.minNeighbourAmp) continue;
        const float angle = sensorAngle(s, cfg);
        fitPoints[fitCount++] = {sinf(angle), cosf(angle), logf(amp[s]), amp[s] * amp[s]};
    }
    out.contributors = fitCount;
    if (fitCount >= 2) {
        const float minCentreDist = cfg.mountRadiusCm + 2.0f;
        for (int iter = 0; iter < 4; iter++) {
            // Weighted Gauss–Newton normal equations: H = JᵀWJ (2×2, symmetric), g = JᵀWr.
            float hBearingBearing = 0, hBearingDist = 0, hDistDist = 0;
            float gBearing = 0, gDist = 0;
            const float distStep = 1e-2f * centreDist;
            for (uint8_t i = 0; i < fitCount; i++) {
                constexpr float bearingStep = 1e-3f;
                const FitPoint& p = fitPoints[i];
                const float predicted = modelLnAmp(p, bearing, centreDist, cfg);
                const float dBearing = (modelLnAmp(p, bearing + bearingStep, centreDist, cfg) - predicted) / bearingStep;
                const float dDist = (modelLnAmp(p, bearing, centreDist + distStep, cfg) - predicted) / distStep;
                const float residual = p.lnAmp - predicted;
                const float w = p.weight;
                hBearingBearing += w * dBearing * dBearing;
                hBearingDist += w * dBearing * dDist;
                hDistDist += w * dDist * dDist;
                gBearing += w * dBearing * residual;
                gDist += w * dDist * residual;
            }
            const float det = hBearingBearing * hDistDist - hBearingDist * hBearingDist;
            if (fabsf(det) < 1e-12f) break;
            bearing += (hDistDist * gBearing - hBearingDist * gDist) / det;
            centreDist += (hBearingBearing * gDist - hBearingDist * gBearing) / det;
            if (bearing > peakAngle + sensorStep) bearing = peakAngle + sensorStep;
            if (bearing < peakAngle - sensorStep) bearing = peakAngle - sensorStep;
            if (centreDist < minCentreDist) centreDist = minCentreDist;
        }
    }

    if (out.clipped) {
        const float clipDist = cfg.mountRadiusCm + cfg.kCm / sqrtf(cfg.clipCode - cfg.baselineCode);
        if (centreDist > clipDist) centreDist = clipDist;
    }
    if (centreDist > cfg.maxCm) centreDist = cfg.maxCm;

    out.angle = Math::wrapRadians(bearing);
    out.distanceCm = centreDist;
    const float ballX = centreDist * sinf(out.angle), ballY = centreDist * cosf(out.angle);
    const float fromPeakX = ballX - cfg.mountRadiusCm * sinf(peakAngle);
    const float fromPeakY = ballY - cfg.mountRadiusCm * cosf(peakAngle);
    out.sensorDistanceCm = sqrtf(fromPeakX * fromPeakX + fromPeakY * fromPeakY);
    out.position = {ballX, ballY};

    const float conf = (amp[peak] - cfg.minAmp) / (4.0f * cfg.minAmp);
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
