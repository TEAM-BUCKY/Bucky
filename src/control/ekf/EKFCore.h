#ifndef BUCKY_EKFCORE_H
#define BUCKY_EKFCORE_H

#include <cstdint>

namespace EKFCore
{
    // Covariance predict step: P = F * P * F' + Qdiag.
    bool predictCovariance(float* P, const float* F, const float* qDiag, uint8_t n);

    // Scalar Kalman update with caller-provided innovation and measurement Jacobian.
    bool updateScalar(float* x, float* P, const float* H, float innovation, float R, uint8_t n);

    // Optimized 2D position update for a 4D state [x, y, vx, vy].
    bool updatePosition2Of4(float x[4],
                            float P[4][4],
                            float zX,
                            float zY,
                            float R,
                            float gate,
                            float* mahalOut,
                            float* likelihoodOut);

    void symmetrize(float* P, uint8_t n);
}

#endif // BUCKY_EKFCORE_H

