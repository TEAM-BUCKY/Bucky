#include "SensorSim.h"
#include "helpers/Math.h"

static constexpr float kCompassPeriod = 0.020f;   // 20ms
static constexpr float kSonarPeriod   = 0.040f;   // 40ms
static constexpr float kIrPeriod      = 0.008f;   // 8ms (125 Hz)
static constexpr float kMaxIrRange    = 2.0f;     // meters
static constexpr float kSensorOffsets[4] = {0.0f, 0.5f * PI_F, PI_F, -0.5f * PI_F};

SensorSim::SensorSim()
    : rng_(42)
{
}

void SensorSim::setNoiseLevel(NoiseLevel level)
{
    noiseLevel_ = level;
}

float SensorSim::gaussian(float stddev)
{
    if (noiseLevel_ == NOISE_NONE || stddev <= 0.0f)
        return 0.0f;

    std::normal_distribution<float> dist(0.0f, stddev);
    return dist(rng_);
}

float SensorSim::rayCastToWall(float rx, float ry, float angle) const
{
    float c = cosf(angle);
    float s = sinf(angle);
    float minD = 10.0f;

    if (fabsf(c) > 1.0e-5f)
    {
        float dxPos = (GroundTruth::X_MAX - rx) / c;
        float dxNeg = (GroundTruth::X_MIN - rx) / c;
        if (dxPos > 0.0f && dxPos < minD) minD = dxPos;
        if (dxNeg > 0.0f && dxNeg < minD) minD = dxNeg;
    }

    if (fabsf(s) > 1.0e-5f)
    {
        float dyPos = (GroundTruth::Y_MAX - ry) / s;
        float dyNeg = (GroundTruth::Y_MIN - ry) / s;
        if (dyPos > 0.0f && dyPos < minD) minD = dyPos;
        if (dyNeg > 0.0f && dyNeg < minD) minD = dyNeg;
    }

    return minD > 9.0f ? 0.0f : minD;
}

SimulatedSensors SensorSim::simulate(const GroundTruth& truth, float dt)
{
    SimulatedSensors out = {};

    float noiseMul = (noiseLevel_ == NOISE_HIGH) ? 3.0f : 1.0f;

    // --- Compass ---
    compassTimer_ += dt;
    if (compassTimer_ >= kCompassPeriod)
    {
        compassTimer_ -= kCompassPeriod;
        float speed = sqrtf(truth.robot.vx * truth.robot.vx + truth.robot.vy * truth.robot.vy);
        float noiseStd = (speed > 0.05f) ? 0.0305f : 0.0049f;
        out.compassRad = truth.robot.theta + gaussian(noiseStd * noiseMul);
        out.compassReady = true;
    }

    // --- Sonar ---
    sonarTimer_ += dt;
    if (sonarTimer_ >= kSonarPeriod)
    {
        sonarTimer_ -= kSonarPeriod;
        out.sonarReady = true;

        for (int i = 0; i < 4; ++i)
        {
            float beamAngle = truth.robot.theta + kSensorOffsets[i];
            float wallDist = rayCastToWall(truth.robot.x, truth.robot.y, beamAngle);

            // Check if an enemy is in the beam path
            float enemyDist = wallDist;
            for (int e = 0; e < 2; ++e)
            {
                if (!truth.enemies[e].active) continue;

                // Project enemy onto beam
                float ex = truth.enemies[e].x - truth.robot.x;
                float ey = truth.enemies[e].y - truth.robot.y;
                float proj = ex * cosf(beamAngle) + ey * sinf(beamAngle);
                float perp = fabsf(-ex * sinf(beamAngle) + ey * cosf(beamAngle));

                // If enemy is in the beam cone (~15cm wide) and closer than wall
                if (proj > 0.0f && perp < 0.15f && proj < enemyDist)
                    enemyDist = proj;
            }

            out.sonarDistanceM[i] = enemyDist + gaussian(0.02f * noiseMul);
            out.sonarDistanceM[i] = fmaxf(0.03f, out.sonarDistanceM[i]);
        }
    }

    // --- IR ball ---
    irTimer_ += dt;
    if (irTimer_ >= kIrPeriod)
    {
        irTimer_ -= kIrPeriod;

        float dx = truth.ball.x - truth.robot.x;
        float dy = truth.ball.y - truth.robot.y;
        float range = sqrtf(dx * dx + dy * dy);

        if (range < kMaxIrRange && range > 0.01f)
        {
            // Check for occlusion by enemies (simple)
            bool occluded = false;
            float ballAngle = atan2f(dy, dx);
            for (int e = 0; e < 2; ++e)
            {
                if (!truth.enemies[e].active) continue;
                float edx = truth.enemies[e].x - truth.robot.x;
                float edy = truth.enemies[e].y - truth.robot.y;
                float eDist = sqrtf(edx * edx + edy * edy);
                if (eDist < range)
                {
                    float eAngle = atan2f(edy, edx);
                    if (fabsf(Math::wrapRadians(eAngle - ballAngle)) < 0.15f)
                        occluded = true;
                }
            }

            if (!occluded)
            {
                // Add noise to bearing and range
                float bearingNoise = gaussian(0.03f * noiseMul);
                float rangeNoise = gaussian(0.02f * range * noiseMul);

                float noisyRange = fmaxf(0.01f, range + rangeNoise);
                float noisyAngle = ballAngle + bearingNoise;

                // Convert to field coordinates (as main.cpp does)
                out.ballFieldX = truth.robot.x + noisyRange * cosf(noisyAngle);
                out.ballFieldY = truth.robot.y + noisyRange * sinf(noisyAngle);
                out.ballRangeM = noisyRange;
                out.ballVisible = true;
            }
        }
    }

    return out;
}
