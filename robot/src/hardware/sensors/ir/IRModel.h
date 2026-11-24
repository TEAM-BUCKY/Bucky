#ifndef ROBOT_IRMODEL_H
#define ROBOT_IRMODEL_H

#include <cmath>

#include <hardware/sensors/ir/IRRing.h>
#include <helpers/math/Geometry.h>
#include <helpers/math/Solvers.h>

// The photodiodes sit heightCm above the ball's centre, so the ball is seen from above at an
// elevation that grows as it comes closer. With cosⁿ optics that costs cosⁿ(elevation): the facing
// sensor's amplitude peaks at the fold distance and falls again inside it, so one amplitude maps to
// a far and a near distance. Range solves pick a branch explicitly.
namespace IRModel
{
    constexpr float MIN_COS = 1e-3f;
    constexpr int RANGE_ITERATIONS = 16;

    inline float lnAmp(const Geometry::Point sensorDir, const Geometry::Point ball, const IRConfig& cfg)
    {
        const Geometry::Point toBall = ball - cfg.mountRadiusCm * sensorDir;
        const float sensorDist = sqrtf(Geometry::dot(toBall, toBall) + cfg.heightCm * cfg.heightCm);
        float cosIncidence = (cosf(cfg.tiltRad) * Geometry::dot(toBall, sensorDir) + sinf(cfg.tiltRad) * cfg.heightCm) / sensorDist;
        if (cosIncidence < MIN_COS) cosIncidence = MIN_COS;
        return 2.0f * logf(cfg.kCm) - 2.0f * logf(sensorDist) + cfg.cosPower * logf(cosIncidence);
    }

    // Centre distance along `bearing` where the sensor facing `sensorDir` peaks (the fold), found
    // as the sign change of the amplitude's slope. Off-axis bearings fold further out.
    inline float foldDistance(const Geometry::Point sensorDir, const float bearing, const IRConfig& cfg)
    {
        const auto slope = [&](const float d) {
            return lnAmp(sensorDir, Geometry::fromBearing(bearing, 1.01f * d), cfg)
                - lnAmp(sensorDir, Geometry::fromBearing(bearing, d), cfg);
        };
        const float lo = cfg.mountRadiusCm + 2.0f;
        if (slope(lo) <= 0.0f) return lo;      // falls from the start: no fold
        return Solvers::bisect(lo, cfg.maxCm, RANGE_ITERATIONS, slope);
    }

    // Centre distance along `bearing` at which the sensor facing `sensorDir` reads `lnPeak`, on the
    // far (lo = fold) or the near (hi = fold) side.
    inline float centreRange(const Geometry::Point sensorDir, const float bearing, const float lnPeak,
                             const float lo, const float hi, const IRConfig& cfg)
    {
        return Solvers::bisect(lo, hi, RANGE_ITERATIONS, [&](const float d) {
            return lnAmp(sensorDir, Geometry::fromBearing(bearing, d), cfg) - lnPeak;
        });
    }

    inline float farFieldOffset(const float lnPeakToNeighbour, const float step, const float cosPower)
    {
        const float target = lnPeakToNeighbour / cosPower;
        if (target <= 0.0f) return step * 0.5f;
        if (target >= -logf(cosf(step))) return 0.0f;
        return Solvers::newton(step * 0.5f - target / step, 0.0f, step * 0.5f, 4,
            [&](const float d) { return logf(cosf(d)) - logf(cosf(step - d)) - target; },
            [&](const float d) { return -tanf(d) - tanf(step - d); });
    }

    inline float clippedPeakOffset(const float lnRightToLeft, const float step, const float cosPower)
    {
        const float target = lnRightToLeft / cosPower;
        return Solvers::newton(0.0f, -step * 0.5f, step * 0.5f, 6,
            [&](const float d) { return logf(cosf(step - d)) - logf(cosf(step + d)) - target; },
            [&](const float d) { return tanf(step - d) + tanf(step + d); });
    }
} // namespace IRModel

#endif //ROBOT_IRMODEL_H
