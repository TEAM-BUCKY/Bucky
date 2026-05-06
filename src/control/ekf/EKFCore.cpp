#include <control/ekf/EKFCore.h>

#include <cmath>
#include <cstdint>

#include "helpers/Math.h"
#include "io/cordic/cordic.h"

namespace EKFCore
{
    void symmetrize(float* __restrict__ P, const uint8_t n)
    {
        for (uint8_t r = 0; r < n; ++r)
        {
            for (uint8_t c = r + 1; c < n; ++c)
            {
                const float avg = 0.5f * (P[r * n + c] + P[c * n + r]);
                P[r * n + c] = avg;
                P[c * n + r] = avg;
            }
        }
    }

    bool predictCovariance(float* __restrict__ P,
                           const float* __restrict__ F,
                           const float* __restrict__ qDiag,
                           const uint8_t n)
    {
        if (P == nullptr || F == nullptr || qDiag == nullptr || n == 0)
            return false;

        float FP[36] = {};
        float newP[36] = {};

        for (uint8_t r = 0; r < n; ++r)
        {
            for (uint8_t c = 0; c < n; ++c)
            {
                float acc = 0.0f;
                for (uint8_t k = 0; k < n; ++k)
                {
                    acc += F[r * n + k] * P[k * n + c];
                }
                FP[r * n + c] = acc;
            }
        }

        for (uint8_t r = 0; r < n; ++r)
        {
            for (uint8_t c = 0; c < n; ++c)
            {
                float acc = 0.0f;
                for (uint8_t k = 0; k < n; ++k)
                {
                    acc += FP[r * n + k] * F[c * n + k];
                }
                newP[r * n + c] = acc;
            }
        }

        for (uint8_t i = 0; i < n; ++i)
            newP[i * n + i] += qDiag[i];

        for (uint8_t r = 0; r < n; ++r)
            for (uint8_t c = 0; c < n; ++c)
                P[r * n + c] = newP[r * n + c];

        // No post-predict symmetrize: F*P*F'+Q is analytically symmetric, and
        // updateScalar always symmetrizes downstream. Any ULP-level drift is
        // corrected on the next sensor update.
        return true;
    }

    bool updateScalar(float* __restrict__ x,
                      float* __restrict__ P,
                      const float* __restrict__ H,
                      const float innovation,
                      const float R,
                      const uint8_t n)
    {
        if (x == nullptr || P == nullptr || H == nullptr || n == 0)
            return false;

        float PHt[6] = {};
        for (uint8_t r = 0; r < n; ++r)
        {
            float acc = 0.0f;
            for (uint8_t k = 0; k < n; ++k)
                acc += P[r * n + k] * H[k];
            PHt[r] = acc;
        }

        float S = R;
        for (uint8_t i = 0; i < n; ++i)
            S += H[i] * PHt[i];

        if (S < 1.0e-9f)
            return false;

        float K[6] = {};
        const float invS = 1.0f / S;
        for (uint8_t i = 0; i < n; ++i)
        {
            K[i] = PHt[i] * invS;
            x[i] += K[i] * innovation;
        }

        float HP[6] = {};
        for (uint8_t c = 0; c < n; ++c)
        {
            float acc = 0.0f;
            for (uint8_t k = 0; k < n; ++k)
                acc += H[k] * P[k * n + c];
            HP[c] = acc;
        }

        for (uint8_t r = 0; r < n; ++r)
            for (uint8_t c = 0; c < n; ++c)
                P[r * n + c] -= K[r] * HP[c];

        symmetrize(P, n);
        return true;
    }

    bool updateScalarPrefix(float* __restrict__ x,
                            float* __restrict__ P,
                            const float* __restrict__ H,
                            const float innovation,
                            const float R,
                            const uint8_t n,
                            const uint8_t m)
    {
        if (x == nullptr || P == nullptr || H == nullptr || n == 0 || m == 0 || m > n)
            return false;

        // H is zero for indices >= m. All inner loops that contract against H
        // iterate only up to m, cutting 2*(n-m)*n MACs per sonar/compass update.
        float PHt[6] = {};
        for (uint8_t r = 0; r < n; ++r)
        {
            float acc = 0.0f;
            for (uint8_t k = 0; k < m; ++k)
                acc += P[r * n + k] * H[k];
            PHt[r] = acc;
        }

        float S = R;
        for (uint8_t i = 0; i < m; ++i)
            S += H[i] * PHt[i];

        if (S < 1.0e-9f)
            return false;

        float K[6] = {};
        const float invS = 1.0f / S;
        for (uint8_t i = 0; i < n; ++i)
        {
            K[i] = PHt[i] * invS;
            x[i] += K[i] * innovation;
        }

        float HP[6] = {};
        for (uint8_t c = 0; c < n; ++c)
        {
            float acc = 0.0f;
            for (uint8_t k = 0; k < m; ++k)
                acc += H[k] * P[k * n + c];
            HP[c] = acc;
        }

        for (uint8_t r = 0; r < n; ++r)
            for (uint8_t c = 0; c < n; ++c)
                P[r * n + c] -= K[r] * HP[c];

        symmetrize(P, n);
        return true;
    }

    bool updatePosition2Of4(float x[4],
                            float P[4][4],
                            const float zX,
                            const float zY,
                            const float R,
                            const float gate,
                            float* mahalOut,
                            float* likelihoodOut)
    {
        const float y0 = zX - x[0];
        const float y1 = zY - x[1];

        const float s00 = P[0][0] + R;
        const float s11 = P[1][1] + R;
        const float s01 = P[0][1];
        const float det = s00 * s11 - s01 * s01;

        if (det < 1.0e-9f)
        {
            if (mahalOut != nullptr) *mahalOut = 99.0f;
            if (likelihoodOut != nullptr) *likelihoodOut = 1.0e-6f;
            return false;
        }

        const float inv00 = s11 / det;
        const float inv11 = s00 / det;
        const float inv01 = -s01 / det;

        const float d2 = y0 * (inv00 * y0 + inv01 * y1) + y1 * (inv01 * y0 + inv11 * y1);
        if (mahalOut != nullptr) *mahalOut = d2;

        const float norm = 1.0f / (2.0f * PI_F * cordic_sqrt(det));
        if (likelihoodOut != nullptr) *likelihoodOut = fmaxf(1.0e-8f, norm * expf(-0.5f * d2));

        if (d2 > gate)
            return false;

        float K[4][2] = {};
        for (uint8_t r = 0; r < 4; ++r)
        {
            const float p0 = P[r][0];
            const float p1 = P[r][1];
            K[r][0] = p0 * inv00 + p1 * inv01;
            K[r][1] = p0 * inv01 + p1 * inv11;
        }

        for (uint8_t r = 0; r < 4; ++r)
            x[r] += K[r][0] * y0 + K[r][1] * y1;

        float HP[2][4] = {};
        for (uint8_t c = 0; c < 4; ++c)
        {
            HP[0][c] = P[0][c];
            HP[1][c] = P[1][c];
        }

        for (uint8_t r = 0; r < 4; ++r)
            for (uint8_t c = 0; c < 4; ++c)
                P[r][c] -= K[r][0] * HP[0][c] + K[r][1] * HP[1][c];

        symmetrize(&P[0][0], 4);
        return true;
    }
}

