#include "IRBallProcessor.h"

#include <cmath>
#include "helpers/Math.h"
#include "io/cordic/cordic.h"

constexpr float minAmplitude = 40.0f;
constexpr float maxAmplitude = 3600.0f;


FORCE_INLINE float safePow(const float value, const float exponent)
{
    return value > 0.0f ? powf(value, exponent) : 0.0f;
}

FORCE_INLINE uint32_t clampSensorCount(const uint32_t sensorCount)
{
    return sensorCount > IRBallProcessor::MAX_SENSORS ? IRBallProcessor::MAX_SENSORS : sensorCount;
}

IRBallProcessor::IRBallProcessor()
{
    // Start with default calibration data.
    reset();
    setDefaultDistanceCalibration();
}

void IRBallProcessor::reset()
{
    // Restore per-sensor gain and baseline defaults. Board 2's idle noise floor
    // sits around 70-90; a baseline of 100 filters that out without swallowing
    // real ball pulses (which typically sit at several hundred to a few thousand).
    for (uint32_t i = 0; i < MAX_SENSORS; ++i)
    {
        gains_[i] = 1.0f;
        baselines_[i] = 100.0f;
    }
}

void IRBallProcessor::setChannelOffset(const uint32_t offset)
{
    channelOffset_ = offset % IR_MUX_CHANNELS;
}

void IRBallProcessor::lockS0AtSensor(const uint8_t peakSensor)
{
    if (s0Locked_) return;
    // Physical sensor i currently reads buffer slot (i + channelOffset_) % 16.
    // To relabel `peakSensor` as the new sensor 0, point the new index 0 at
    // the same buffer slot that peakSensor is on today.
    channelOffset_ = (static_cast<uint32_t>(peakSensor) + channelOffset_) % IR_MUX_CHANNELS;
    // Reset the drift detector so it doesn't spend the next 8 frames
    // pulling the offset back toward the boot-calibration value.
    driftFrames_ = 0;
    pendingOffset_ = channelOffset_;
    s0Locked_ = true;
}

void IRBallProcessor::setThresholdRatio(const float ratio)
{
    // Keep the threshold ratio in a safe range.
    thresholdRatio_ = clampf(ratio, 0.0f, 0.95f);
}

void IRBallProcessor::setSensorCalibration(const float* gains, const float* baselines, const uint32_t count)
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

void IRBallProcessor::setDistanceCalibration(const float* amplitudes, const float* distances, const uint32_t count)
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

void IRBallProcessor::setDefaultDistanceCalibration(float minDistanceCm, float maxDistanceCm)
{
    // Build a simple default amplitude-to-distance curve.
    minDistanceCm = clampf(minDistanceCm, 1.0f, maxDistanceCm);
    maxDistanceCm = clampf(maxDistanceCm, minDistanceCm + 1.0f, 1000.0f);

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

float IRBallProcessor::lookupDistanceFromAmplitude(const float* amplitudes, const float* distances, const uint32_t count,
                                                 const float amplitude)
{
    // Interpolate the distance from the calibrated amplitude table. The LUT
    // is sorted with amplitudes descending (distances ascending).
    if (count == 0)
        return 0.0f;

    if (amplitude >= amplitudes[0])
        return distances[0];

    if (amplitude <= amplitudes[count - 1])
        return distances[count - 1];

    // Binary search for the first index i where amplitudes[i] <= amplitude.
    // Since the array is descending, this is the lower end of the bracketing
    // segment. Replaces the prior O(n) linear scan with O(log n).
    uint32_t lo = 1;
    uint32_t hi = count - 1;
    while (lo < hi)
    {
        const uint32_t mid = lo + ((hi - lo) >> 1);
        if (amplitudes[mid] <= amplitude)
            hi = mid;
        else
            lo = mid + 1;
    }

    const uint32_t i = lo;
    const float a0 = amplitudes[i - 1];
    const float a1 = amplitudes[i];
    const float d0 = distances[i - 1];
    const float d1 = distances[i];
    const float span = a0 - a1;
    const float t = span > 0.0f ? (a0 - amplitude) / span : 0.0f;
    return d0 + t * (d1 - d0);
}

IRBallObservation IRBallProcessor::process(const uint16_t* raw, const uint32_t sensorCount) const
{
    // Turn raw sweeps into a single ball observation.
    IRBallObservation obs;
    obs.sensorCount = sensorCount;

    if (raw == nullptr || sensorCount == 0)
        return obs;

    const uint32_t n = clampSensorCount(sensorCount);
    float calibrated[MAX_SENSORS] = {};

    // --- Runtime mux-phase drift detection ---
    // The 74HC4040 mux counter is free-running and its CLK line can pick up
    // a glitch edge, rotating the buffer-index-to-physical-sensor mapping.
    // ir_calibrate_channels() runs once at boot, so without this block a
    // drift event misroutes every sensor for the rest of the session.
    //
    // The detection mirrors boot calibration: find the `unconnectedCount`
    // consecutive slots with the smallest per-channel max, and place sensor 0
    // right after that cluster. This is only reliable when the ring is
    // otherwise quiet — if a ball is illuminating a bright arc, the 4
    // back-facing connected sensors on the opposite side can read lower
    // than floating unconnected mux inputs picking up noise, causing the
    // detector to latch onto a 180°-rotated (wrong) offset. So gate the
    // detection on `peakMax < kDriftDetectPeakMax` — same "field clear"
    // precondition the header comment on ir_calibrate_channels spells out.
    if (n < IR_MUX_CHANNELS && n > 0)
    {
        const uint32_t unconnectedCount = IR_MUX_CHANNELS - n;

        uint16_t maxPerCh[IR_MUX_CHANNELS] = {};
        for (uint32_t sweep = 0; sweep < IR_SWEEPS_PER_CYCLE; ++sweep)
        {
            const uint16_t* row = raw + sweep * IR_MUX_CHANNELS;
            for (uint32_t ch = 0; ch < IR_MUX_CHANNELS; ++ch)
                if (row[ch] > maxPerCh[ch]) maxPerCh[ch] = row[ch];
        }

        uint16_t peakMax = 0;
        for (uint32_t ch = 0; ch < IR_MUX_CHANNELS; ++ch)
            if (maxPerCh[ch] > peakMax) peakMax = maxPerCh[ch];

        if (peakMax < kDriftDetectPeakMax)
        {
            uint32_t bestSum = UINT32_MAX;
            uint32_t secondBestSum = UINT32_MAX;
            uint8_t  bestStart = 0;
            for (uint8_t s = 0; s < IR_MUX_CHANNELS; ++s)
            {
                uint32_t sum = 0;
                for (uint32_t k = 0; k < unconnectedCount; ++k)
                    sum += maxPerCh[(s + k) % IR_MUX_CHANNELS];
                if (sum < bestSum)
                {
                    secondBestSum = bestSum;
                    bestSum = sum;
                    bestStart = s;
                }
                else if (sum < secondBestSum)
                {
                    secondBestSum = sum;
                }
            }

            // Confidence gate: runner-up must be >1.5× + 10 counts above best.
            // Rotating a genuine quiet-cluster window by one slot replaces one
            // unconnected channel with one connected channel, so the true
            // phase has runner-up ≈ 2-3× best. A close runner-up means the
            // detection is ambiguous — skip it.
            if (secondBestSum > bestSum + bestSum / 2 + 10)
            {
                const uint32_t detected =
                    (static_cast<uint32_t>(bestStart) + unconnectedCount) % IR_MUX_CHANNELS;

                if (detected == channelOffset_)
                {
                    driftFrames_ = 0;
                }
                else if (detected == pendingOffset_)
                {
                    if (++driftFrames_ >= kDriftConfirmFrames)
                    {
                        channelOffset_ = detected;
                        driftFrames_ = 0;
                    }
                }
                else
                {
                    pendingOffset_ = detected;
                    driftFrames_ = 1;
                }
            }
        }
        else
        {
            // Ball-bright frame — detection unreliable, reset any pending flip.
            driftFrames_ = 0;
        }
    }

    // Rebuild the sensor-angle sin/cos tables only when n changes. For a
    // fixed board that's once at boot; steady-state cost is zero. Prior code
    // called cordic_sin_cos + degrees→radians inside the per-sensor loop.
    //
    // Ring geometry on this board: sensor 0 is mounted at body +Y (front of
    // the robot) and indices increase clockwise, so sensor i sits at physical
    // body angle (π/2 − 2π·i/N) in standard CCW math convention. This makes
    // the weighted-centroid atan2 emit a bearing where 0 = body +X (right),
    // +π/2 = body +Y (forward), +π = body −X (left) — matching how main.cpp
    // and the FSM interpret `bearingDeg` downstream (`alpha = theta + bearing`
    // produces the correct world-frame ball position at any self.theta).
    if (n != cachedSensorCount_)
    {
        const float kTwoPiOverN = (n > 0) ? (2.0f * PI_F / static_cast<float>(n)) : 0.0f;
        for (uint32_t i = 0; i < n; ++i)
            cordic_sin_cos(PI_F * 0.5f - kTwoPiOverN * static_cast<float>(i),
                           &sinAngle_[i], &cosAngle_[i]);
        cachedSensorCount_ = n;
    }

    float peak = 0.0f;
    uint8_t peakIndex = 0;
    for (uint32_t i = 0; i < n; ++i)
    {
        // Board 2 convention: raw ADC IS the pulse amplitude (0 = idle,
        // higher = stronger IR return). Take the max across sweeps.
        // channelOffset_ remaps physical sensor i to its actual mux slot.
        const uint32_t bufCh = (i + channelOffset_) % IR_MUX_CHANNELS;
        float value = 0.0f;
        for (uint32_t sweep = 0; sweep < IR_SWEEPS_PER_CYCLE; ++sweep)
        {
            const uint32_t idx = sweep * IR_MUX_CHANNELS + bufCh;
            value = fmaxf(value, static_cast<float>(raw[idx]));
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
        sx += w * cosAngle_[i];
        sy += w * sinAngle_[i];
        totalWeight += w;
    }

    if (!(totalWeight > 0.0f))
        return obs;

    float angleRad, vectorMagnitude;
    cordic_atan2_mod(sy, sx, &angleRad, &vectorMagnitude);
    // Average the strongest sensors for a rough signal strength.
    const float strength = (top1 + top2 + top3) / 3.0f;
    const float rangeCm = lookupDistanceFromAmplitude(amplitudeLut_, distanceLut_, lutCount_, strength);

    obs.valid = true;
    obs.peakSensor = peakIndex;
    obs.bearingDeg = Math::wrapDegrees(Math::radiansToDegrees(angleRad));
    obs.rangeCm = rangeCm;
    obs.strength = strength;
    obs.confidence = clampf((vectorMagnitude / totalWeight) * (strength / (strength + 250.0f)), 0.0f, 1.0f);

    // atan2's output is already in [-π, π], so skip the deg→rad round-trip
    // and use angleRad directly for x/y. bearingDeg above is retained for
    // the public observation API (consumers expect [0, 360)).
    float sinB, cosB;
    cordic_sin_cos(angleRad, &sinB, &cosB);
    obs.x = rangeCm * cosB;
    obs.y = rangeCm * sinB;

    return obs;
}