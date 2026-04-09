#include "IRBallTracker.h"

#include <cmath>
#include "../io/cordic/cordic.h"
#include "helpers/Math.h"

constexpr float minAmplitude = 40.0f;
constexpr float maxAmplitude = 3600.0f;


FORCE_INLINE float safePow(const float value, const float exponent)
{
    return value > 0.0f ? powf(value, exponent) : 0.0f;
}

FORCE_INLINE uint32_t clampSensorCount(const uint32_t sensorCount)
{
    return sensorCount > IRBallTracker::MAX_SENSORS ? IRBallTracker::MAX_SENSORS : sensorCount;
}

IRBallTracker::IRBallTracker()
{
    // Start with default calibration data.
    reset();
    setDefaultDistanceCalibration();
}

void IRBallTracker::reset()
{
    // Restore per-sensor gain and baseline defaults.
    for (uint32_t i = 0; i < MAX_SENSORS; ++i)
    {
        gains_[i] = 1.0f;
        baselines_[i] = 0.0f;
    }
}

void IRBallTracker::setThresholdRatio(const float ratio)
{
    // Keep the threshold ratio in a safe range.
    thresholdRatio_ = Math::clampf(ratio, 0.0f, 0.95f);
}

void IRBallTracker::setSensorCalibration(const float* gains, const float* baselines, const uint32_t count)
{
    if (gains != nullptr)
    {
        // Apply per-sensor gain calibration.
        const uint32_t n = clampSensorCount(count);
        for (uint32_t i = 0; i < n; ++i)
        {
            gains_[i] = gains[i] > 0.0f ? gains[i] : 1.0f;
        }
    }

    if (baselines != nullptr)
    {
        // Apply per-sensor baseline offsets.
        const uint32_t n = clampSensorCount(count);
        for (uint32_t i = 0; i < n; ++i)
        {
            baselines_[i] = baselines[i] > 0.0f ? baselines[i] : 0.0f;
        }
    }
}

void IRBallTracker::setDistanceCalibration(const float* amplitudes, const float* distances, const uint32_t count)
{
    // Load a custom amplitude-to-distance table.
    const uint32_t n = count > LUT_SIZE ? LUT_SIZE : count;
    if (amplitudes == nullptr || distances == nullptr || n == 0)
    {
        setDefaultDistanceCalibration();
        return;
    }

    for (uint32_t i = 0; i < n; ++i)
    {
        amplitudeLut_[i] = amplitudes[i];
        distanceLut_[i] = distances[i];
    }
    lutCount_ = n;

    for (uint32_t i = 1; i < lutCount_; ++i)
    {
        const float a = amplitudeLut_[i];
        const float d = distanceLut_[i];
        int32_t j = static_cast<int32_t>(i) - 1;
        while (j >= 0 && distanceLut_[j] > d)
        {
            amplitudeLut_[j + 1] = amplitudeLut_[j];
            distanceLut_[j + 1] = distanceLut_[j];
            --j;
        }
        amplitudeLut_[j + 1] = a;
        distanceLut_[j + 1] = d;
    }
}

void IRBallTracker::setDefaultDistanceCalibration(float minDistanceCm, float maxDistanceCm)
{
    // Build a simple default amplitude-to-distance curve.
    minDistanceCm = Math::clampf(minDistanceCm, 1.0f, maxDistanceCm);
    maxDistanceCm = Math::clampf(maxDistanceCm, minDistanceCm + 1.0f, 1000.0f);

    for (uint32_t i = 0; i < LUT_SIZE; ++i)
    {
        const float t = static_cast<float>(i) / static_cast<float>(LUT_SIZE - 1);
        const float distance = minDistanceCm + t * (maxDistanceCm - minDistanceCm);
        const float normalized = minDistanceCm / distance;
        const float amplitude = minAmplitude + (maxAmplitude - minAmplitude) * safePow(normalized, 1.8f);
        distanceLut_[i] = distance;
        amplitudeLut_[i] = amplitude;
    }

    lutCount_ = LUT_SIZE;
}

float IRBallTracker::lookupDistanceFromAmplitude(const float* amplitudes, const float* distances, const uint32_t count,
                                                 const float amplitude)
{
    // Interpolate the distance from the calibrated amplitude table.
    if (count == 0)
        return 0.0f;

    if (amplitude >= amplitudes[0])
        return distances[0];

    if (amplitude <= amplitudes[count - 1])
        return distances[count - 1];

    for (uint32_t i = 1; i < count; ++i)
    {
        if (amplitude >= amplitudes[i])
        {
            const float a0 = amplitudes[i - 1];
            const float a1 = amplitudes[i];
            const float d0 = distances[i - 1];
            const float d1 = distances[i];
            const float span = a0 - a1;
            const float t = span > 0.0f ? (a0 - amplitude) / span : 0.0f;
            return d0 + t * (d1 - d0);
        }
    }

    return distances[count - 1];
}

IRBallObservation IRBallTracker::process(const uint16_t* raw, const uint32_t sensorCount) const
{
    // Turn raw sweeps into a single ball observation.
    IRBallObservation obs;
    obs.sensorCount = sensorCount;

    if (raw == nullptr || sensorCount == 0)
        return obs;

    const uint32_t n = clampSensorCount(sensorCount);
    float calibrated[MAX_SENSORS] = {};

    float peak = 0.0f;
    uint8_t peakIndex = 0;
    for (uint32_t i = 0; i < n; ++i)
    {
        float value = 0.0f;
        for (uint32_t sweep = 0; sweep < IR_SWEEPS_PER_CYCLE; ++sweep)
        {
            const uint32_t idx = sweep * IR_MUX_CHANNELS + i;
            value = fmaxf(value, raw[idx]);
        }

        value = (value - baselines_[i]) * gains_[i];
        if (value < 0.0f) value = 0.0f;
        calibrated[i] = value;

        if (value > peak)
        {
            peak = value;
            peakIndex = static_cast<uint8_t>(i);
        }
    }

    if (!(peak > 0.0f))
        return obs;

    // Keep only readings above the peak-based threshold.
    const float threshold = peak * thresholdRatio_;
    float totalWeight = 0.0f;
    float sx = 0.0f;
    float sy = 0.0f;
    float top1 = 0.0f, top2 = 0.0f, top3 = 0.0f;

    for (uint32_t i = 0; i < n; ++i)
    {
        const float value = calibrated[i] > threshold ? calibrated[i] - threshold : 0.0f;
        if (value <= 0.0f)
            continue;

        if (value > top1)
        {
            top3 = top2;
            top2 = top1;
            top1 = value;
        }
        else if (value > top2)
        {
            top3 = top2;
            top2 = value;
        }
        else if (value > top3)
            top3 = value;

        const float w = value * value;
        const float angle = Math::degreesToRadians(360.0f * static_cast<float>(i) / static_cast<float>(n));
        // Convert each active sensor into a unit vector contribution.
        float s = 0.0f;
        float c = 0.0f;
        cordic_sin_cos(angle, &s, &c);
        sx += w * c;
        sy += w * s;
        totalWeight += w;
    }

    if (!(totalWeight > 0.0f))
        return obs;

    const float angleRad = cordic_atan2(sy, sx);
    const float vectorMagnitude = sqrtf(sx * sx + sy * sy);
    // Average the strongest sensors for a rough signal strength.
    const float strength = (top1 + top2 + top3) / 3.0f;
    const float rangeCm = lookupDistanceFromAmplitude(amplitudeLut_, distanceLut_, lutCount_, strength);

    obs.valid = true;
    obs.peakSensor = peakIndex;
    obs.bearingDeg = Math::wrapDegrees(Math::radiansToDegrees(angleRad));
    obs.rangeCm = rangeCm;
    obs.strength = strength;
    obs.confidence = Math::clampf((vectorMagnitude / totalWeight) * (strength / (strength + 250.0f)), 0.0f, 1.0f);

    // Convert the final bearing and range into Cartesian coordinates.
    float sinB = 0.0f;
    float cosB = 0.0f;
    cordic_sin_cos(Math::degreesToRadians(obs.bearingDeg), &sinB, &cosB);
    obs.x = rangeCm * cosB;
    obs.y = rangeCm * sinB;

    return obs;
}