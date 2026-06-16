#include "MatrixMultiplyArm.h"
#include "optimizations/optimizations.h"

namespace ArmMatrix
{
    namespace
    {

        FORCE_INLINE float dot4Scalar(const float* lhs, const float* rhs)
        {
            return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2] + lhs[3] * rhs[3];
        }

        FORCE_INLINE float dot2Scalar(const float* lhs, const float* rhs)
        {
            return lhs[0] * rhs[0] + lhs[1] * rhs[1];
        }
    }

    FORCE_INLINE void multiply4x4(const float lhs[4][4], const float rhs[4][4], float out[4][4])
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                out[i][j] = lhs[i][0] * rhs[0][j] + lhs[i][1] * rhs[1][j] + lhs[i][2] * rhs[2][j] + lhs[i][3] * rhs[3][j];
            }
        }
    }

    FORCE_INLINE void multiply4x4ByTransposed(const float lhs[4][4], const float rhs[4][4], float out[4][4])
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                out[i][j] = dot4Scalar(lhs[i], rhs[j]);
            }
        }
    }

    FORCE_INLINE void multiply4x2ByTransposed(const float lhs[4][4], const float rhs[2][4], float out[4][2])
    {
        for (int i = 0; i < 4; ++i)
        {
            out[i][0] = dot4Scalar(lhs[i], rhs[0]);
            out[i][1] = dot4Scalar(lhs[i], rhs[1]);
        }
    }

    FORCE_INLINE void multiply2x2(const float lhs[2][4], const float rhs[4][2], float out[2][2])
    {
        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < 2; ++j)
            {
                out[i][j] = lhs[i][0] * rhs[0][j] + lhs[i][1] * rhs[1][j] + lhs[i][2] * rhs[2][j] + lhs[i][3] * rhs[3][j];
            }
        }
    }

    FORCE_INLINE void multiply4x2(const float lhs[4][2], const float rhs[2][2], float out[4][2])
    {
        const float rhsCol0[2] = {rhs[0][0], rhs[1][0]};
        const float rhsCol1[2] = {rhs[0][1], rhs[1][1]};

        for (int i = 0; i < 4; ++i)
        {
            out[i][0] = dot2Scalar(lhs[i], rhsCol0);
            out[i][1] = dot2Scalar(lhs[i], rhsCol1);
        }
    }

    FORCE_INLINE void multiply4x4From4x2And2x4(const float lhs[4][2], const float rhs[2][4], float out[4][4])
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                out[i][j] = lhs[i][0] * rhs[0][j] + lhs[i][1] * rhs[1][j];
            }
        }
    }

    FORCE_INLINE void multiply4x2Vector(const float lhs[4][2], const float rhs[2], float out[4])
    {
        for (int i = 0; i < 4; ++i)
        {
            out[i] = dot2Scalar(lhs[i], rhs);
        }
    }
}
