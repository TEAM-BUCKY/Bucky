#ifndef BUCKY_MATRIX_MULTIPLY_ARM_H
#define BUCKY_MATRIX_MULTIPLY_ARM_H

namespace ArmMatrix
{
    void multiply4x4(const float lhs[4][4], const float rhs[4][4], float out[4][4]);
    void multiply4x4ByTransposed(const float lhs[4][4], const float rhs[4][4], float out[4][4]);
    void multiply4x2ByTransposed(const float lhs[4][4], const float rhs[2][4], float out[4][2]);
    void multiply2x2(const float lhs[2][4], const float rhs[4][2], float out[2][2]);
    void multiply4x2(const float lhs[4][2], const float rhs[2][2], float out[4][2]);
    void multiply4x4From4x2And2x4(const float lhs[4][2], const float rhs[2][4], float out[4][4]);
    void multiply4x2Vector(const float lhs[4][2], const float rhs[2], float out[4]);
} // namespace ArmMatrix

#endif // BUCKY_MATRIX_MULTIPLY_ARM_H

