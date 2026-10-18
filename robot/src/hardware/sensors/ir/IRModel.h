#ifndef ROBOT_IRMODEL_H
#define ROBOT_IRMODEL_H

#include <cmath>

#include <hardware/sensors/ir/IRRing.h>
#include <helpers/math/Geometry.h>
#include <helpers/math/Solvers.h>

namespace IRModel
{
    constexpr float MIN_COS = 1e-3f;

    FORCE_INLINE float inverseSquareRange(const float amp, const IRConfig& cfg)
    {
        return cfg.kCm / sqrtf(amp);
    }

    /**
     * On-axis amplitude of a reading taken `offset` radians off the sensor axis.
     * @param amp The amplitude reading from the sensor.
     * @param offset The offset from the sensor axis in radians.
     * @param cfg The IR configuration.
     * @return The on-axis amplitude.
     */
    FORCE_INLINE float onAxisAmp(const float amp, const float offset, const IRConfig& cfg)
    {
        const float cosOffset = cosf(offset);
        return amp / powf(cosOffset > MIN_COS ? cosOffset : MIN_COS, cfg.cosPower);
    }

    /**
     * Calculates the natural logarithm of the amplitude for a given sensor direction and ball position.
     * @param sensorDir The direction of the sensor as a point.
     * @param ball The position of the ball as a point.
     * @param cfg The IR configuration.
     * @return The natural logarithm of the amplitude.
     */
    inline float lnAmp(const Geometry::Point sensorDir, const Geometry::Point ball, const IRConfig& cfg)
    {
        const Geometry::Point toBall = ball - cfg.mountRadiusCm * sensorDir;
        const float sensorDist = Geometry::length(toBall);
        float cosIncidence = Geometry::dot(toBall, sensorDir) / sensorDist;
        if (cosIncidence < MIN_COS) cosIncidence = MIN_COS;
        return 2.0f * logf(cfg.kCm) - 2.0f * logf(sensorDist) + cfg.cosPower * logf(cosIncidence);
    }

    /**
     * Calculates the offset of a peak in the far field based on the natural logarithm of the amplitude ratio between the peak and its neighbour.
     * @param lnPeakToNeighbour The natural logarithm of the amplitude ratio between the peak and its neighbour.
     * @param step The step size for the search.
     * @param cosPower The cosine power for the calculation.
     * @return The offset of the peak.
     */
    inline float farFieldOffset(const float lnPeakToNeighbour, const float step, const float cosPower)
    {
        const float target = lnPeakToNeighbour / cosPower;
        if (target <= 0.0f) return step * 0.5f;
        if (target >= -logf(cosf(step))) return 0.0f;
        return Solvers::newton(step * 0.5f - target / step, 0.0f, step * 0.5f, 4,
            [&](const float d) { return logf(cosf(d)) - logf(cosf(step - d)) - target; },
            [&](const float d) { return -tanf(d) - tanf(step - d); });
    }

    /**
     * Calculates the offset of a clipped peak based on the natural logarithm of the amplitude ratio between the right and left neighbours.
     * @param lnRightToLeft The natural logarithm of the amplitude ratio between the right and left neighbours.
     * @param step The step size for the search.
     * @param cosPower The cosine power for the calculation.
     * @return The offset of the clipped peak.
     */
    inline float clippedPeakOffset(const float lnRightToLeft, const float step, const float cosPower)
    {
        const float target = lnRightToLeft / cosPower;
        return Solvers::newton(0.0f, -step * 0.5f, step * 0.5f, 6,
            [&](const float d) { return logf(cosf(step - d)) - logf(cosf(step + d)) - target; },
            [&](const float d) { return tanf(step - d) + tanf(step + d); });
    }
} // namespace IRModel

#endif //ROBOT_IRMODEL_H
