#include "cordic.h"
#include "../bitboard/bitboard.h"
#include <cmath>

static constexpr float Q31_SCALE = 2147483648.0f;  // 2^31
static constexpr float Q31_INV   = 1.0f / Q31_SCALE;
static constexpr float PI_F      = 3.14159265358979f;
static constexpr float INV_PI    = 1.0f / PI_F;

// 6 iterations ≈ 20-bit accuracy, completes in ~7 AHB cycles
static constexpr uint32_t DEFAULT_PRECISION = 6;

static inline int32_t to_q31(const float x) {
    return (x >= 1.0f) ? 0x7FFFFFFF : static_cast<int32_t>(x * Q31_SCALE);
}

static inline float from_q31(const int32_t x) {
    return static_cast<float>(x) * Q31_INV;
}

void cordic_init() {
    setMask(RCC->AHB1ENR, RCC_AHB1ENR_CORDICEN);
}

void cordic_sin_cos(const float angle_rad, float& sin_out, float& cos_out) {
    // Normalize angle to [-1, 1) representing [-π, π) for q1.31 input
    float n = angle_rad * INV_PI;
    n -= 2.0f * floorf((n + 1.0f) * 0.5f);

    // Cosine function, 1 argument (angle), 2 results (cos then sin)
    CORDIC->CSR = (CordicFunc::COSINE << CORDIC_CSR_FUNC_Pos)
               | (DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos)
               | CORDIC_CSR_NRES;

    CORDIC->WDATA = static_cast<uint32_t>(to_q31(n));

    while (!(CORDIC->CSR & CORDIC_CSR_RRDY)) {}
    cos_out = from_q31(static_cast<int32_t>(CORDIC->RDATA));
    sin_out = from_q31(static_cast<int32_t>(CORDIC->RDATA));
}

float cordic_sin(const float angle_rad) {
    float s, c;
    cordic_sin_cos(angle_rad, s, c);
    return s;
}

float cordic_cos(const float angle_rad) {
    float s, c;
    cordic_sin_cos(angle_rad, s, c);
    return c;
}

float cordic_atan2(const float y, const float x) {
    if (x == 0.0f && y == 0.0f) return 0.0f;

    // Normalize both inputs to q1.31 range [-1, 1)
    const float m = fmaxf(fabsf(x), fabsf(y));
    const float inv = 1.0f / m;

    // Phase function, 2 arguments (x then y), 1 result (angle)
    CORDIC->CSR = (CordicFunc::PHASE << CORDIC_CSR_FUNC_Pos)
               | (DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos)
               | CORDIC_CSR_NARGS;

    CORDIC->WDATA = static_cast<uint32_t>(to_q31(x * inv));
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(y * inv));

    // Result is atan2(y, x) / π in q1.31
    while (!(CORDIC->CSR & CORDIC_CSR_RRDY)) {}
    return from_q31(static_cast<int32_t>(CORDIC->RDATA)) * PI_F;
}
