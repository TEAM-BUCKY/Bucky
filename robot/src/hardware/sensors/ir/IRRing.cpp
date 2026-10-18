#include "IRRing.h"

#include <cmath>

#include <hardware/sensors/ir/IRModel.h>
#include <helpers/math/Geometry.h>
#include <helpers/math/Math.h>

namespace
{
    struct RingFrame {
        float amp[GPort::SENSORS] = {};
        bool clipped[GPort::SENSORS] = {};
        float step = 0.0f;
        uint8_t peak = 0, left = 0, right = 0;
    };

    struct FitPoints {
        Geometry::Point dir[3];
        Solvers::Observation lnAmp[3];
        uint8_t count = 0;
    };

    RingFrame readRing(const uint16_t frame[GPort::SENSORS], const uint8_t sensorCount, const IRConfig& cfg)
    {
        RingFrame ring;
        ring.step = 2.0f * _PI_F / static_cast<float>(sensorCount);
        for (uint8_t s = 0; s < sensorCount; s++) {
            const float aboveBaseline = static_cast<float>(frame[s]) - cfg.baselineCode;
            ring.amp[s] = aboveBaseline > 0.0f ? aboveBaseline : 0.0f;
            ring.clipped[s] = frame[s] >= cfg.clipCode;
            if (ring.amp[s] > ring.amp[ring.peak]) ring.peak = s;
        }
        ring.left = Math::ringPrev(ring.peak, sensorCount);
        ring.right = Math::ringNext(ring.peak, sensorCount);
        return ring;
    }

    float peakOffset(const RingFrame& ring, const IRConfig& cfg)
    {
        const float* amp = ring.amp;
        if (!ring.clipped[ring.peak]) {
            const bool rightStronger = amp[ring.right] >= amp[ring.left];
            const uint8_t neighbour = rightStronger ? ring.right : ring.left;
            if (amp[neighbour] < cfg.minNeighbourAmp) return 0.0f;
            const float delta = IRModel::farFieldOffset(logf(amp[ring.peak] / amp[neighbour]), ring.step, cfg.cosPower);
            return rightStronger ? delta : -delta;
        }
        if (amp[ring.left] < cfg.minNeighbourAmp || amp[ring.right] < cfg.minNeighbourAmp) return 0.0f;
        return IRModel::clippedPeakOffset(logf(amp[ring.right] / amp[ring.left]), ring.step, cfg.cosPower);
    }

    FitPoints collectFitPoints(const RingFrame& ring, const IRConfig& cfg)
    {
        FitPoints points;
        const uint8_t candidates[3] = {ring.left, ring.peak, ring.right};
        for (const uint8_t s : candidates) {
            const float amp = ring.amp[s];
            if (ring.clipped[s] || amp < cfg.minNeighbourAmp) continue;
            points.dir[points.count] = Geometry::fromBearing(IRRing::sensorAngle(s, cfg), 1.0f);
            points.lnAmp[points.count] = {logf(amp), amp * amp};
            points.count++;
        }
        return points;
    }

    float limitDistance(float centreDist, const bool clipped, const IRConfig& cfg)
    {
        if (clipped) {
            const float clipDist = cfg.mountRadiusCm + IRModel::inverseSquareRange(cfg.clipCode - cfg.baselineCode, cfg);
            if (centreDist > clipDist) centreDist = clipDist;
        }
        return centreDist > cfg.maxCm ? cfg.maxCm : centreDist;
    }
}

float IRRing::sensorAngle(const uint8_t sensor, const IRConfig& cfg)
{
    return 2.0f * _PI_F * static_cast<float>(sensor) / static_cast<float>(cfg.sensors);
}

float IRRing::sensorDistanceCm(const float amp, const IRConfig& cfg)
{
    if (amp <= 0.0f) return cfg.maxCm;
    const float dist = IRModel::inverseSquareRange(amp, cfg);
    return dist > cfg.maxCm ? cfg.maxCm : dist;
}

BallReading IRRing::estimate(const uint16_t frame[GPort::SENSORS], const IRConfig& cfg)
{
    BallReading out;
    const uint8_t sensorCount = cfg.sensors > GPort::SENSORS ? GPort::SENSORS : cfg.sensors;
    if (sensorCount < 3) return out;

    const RingFrame ring = readRing(frame, sensorCount, cfg);
    out.peakSensor = ring.peak;
    out.peakAmp = ring.amp[ring.peak];
    if (out.peakAmp < cfg.minAmp) return out;

    out.seen = true;
    out.clipped = ring.clipped[ring.peak];

    const float peakAngle = sensorAngle(ring.peak, cfg);
    const float offset = peakOffset(ring, cfg);
    float bearing = peakAngle + offset;
    float centreDist = cfg.mountRadiusCm + IRModel::inverseSquareRange(IRModel::onAxisAmp(out.peakAmp, offset, cfg), cfg);

    const auto [dir, lnAmp, count] = collectFitPoints(ring, cfg);
    out.contributors = count;
    if (count >= 2) {
        const float minCentreDist = cfg.mountRadiusCm + 2.0f;
        Solvers::gaussNewton2(bearing, centreDist, 4, lnAmp, count,
            [&](const int i, const float b, const float d) { return IRModel::lnAmp(dir[i], Geometry::fromBearing(b, d), cfg); },
            1e-3f, [](const float d) { return 1e-2f * d; },
            [&](float& b, float& d) {
                b = clamp(b, peakAngle - ring.step, peakAngle + ring.step);
                d = fmaxf(d, minCentreDist);
            });
    }

    out.angle = Math::wrapRadians(bearing);
    out.distanceCm = limitDistance(centreDist, out.clipped, cfg);

    const Geometry::Point ball = Geometry::fromBearing(out.angle, out.distanceCm);
    out.sensorDistanceCm = Geometry::length(ball - Geometry::fromBearing(peakAngle, cfg.mountRadiusCm));
    out.position = {ball.x, ball.y};
    out.confidence = (out.peakAmp - cfg.minAmp) / (4.0f * cfg.minAmp);

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
