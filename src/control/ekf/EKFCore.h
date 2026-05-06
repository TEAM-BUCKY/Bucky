#ifndef BUCKY_EKFCORE_H
#define BUCKY_EKFCORE_H

#include <cstdint>

namespace EKFCore
{
    // Covariance predict step: P = F * P * F' + Qdiag.
    bool predictCovariance(float* __restrict__ P,
                           const float* __restrict__ F,
                           const float* __restrict__ qDiag,
                           uint8_t n);

    // Scalar Kalman update with caller-provided innovation and measurement Jacobian.
    bool updateScalar(float* __restrict__ x,
                      float* __restrict__ P,
                      const float* __restrict__ H,
                      float innovation, float R, uint8_t n);

    // Scalar update where H is known to be zero for indices >= m (m <= n).
    // All dot products with H run only over the first m components; the state
    // update and P decrement still span the full n dimensions.
    bool updateScalarPrefix(float* __restrict__ x,
                            float* __restrict__ P,
                            const float* __restrict__ H,
                            float innovation, float R, uint8_t n, uint8_t m);

    // Optimized 2D position update for a 4D state [x, y, vx, vy].
    bool updatePosition2Of4(float x[4],
                            float P[4][4],
                            float zX,
                            float zY,
                            float R,
                            float gate,
                            float* mahalOut,
                            float* likelihoodOut);

    void symmetrize(float* __restrict__ P, uint8_t n);
}

#endif // BUCKY_EKFCORE_H

