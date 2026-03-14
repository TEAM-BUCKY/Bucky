#include "cordic.h"
#include "../bitboard/bitboard.h"
#include <cmath>

#include "optimizations/optimizations.h"

static constexpr float Q31_SCALE = 2147483648.0f;
static constexpr float Q31_INV = 1.0f / Q31_SCALE;
static constexpr float LN2 = 0.69314718f;

static constexpr uint32_t DEFAULT_PRECISION = 6;

static FORCE_INLINE int32_t to_q31(const float x) {
    return x >= 1.0f ? 0x7FFFFFFF : static_cast<int32_t>(x * Q31_SCALE);
}

static FORCE_INLINE float from_q31(const int32_t x) {
    return static_cast<float>(x) * Q31_INV;
}

static FORCE_INLINE float from_uq31(const uint32_t x) {
    return static_cast<float>(x) * Q31_INV;
}

void cordic_poll()
{
    while (__builtin_expect(!(CORDIC->CSR & CORDIC_CSR_RRDY), 1)) { }
}

void cordic_sin_cos(const float angle_rad, float &sin_out, float &cos_out) {
    float n = angle_rad * INV_PI;
    n -= 2.0f * floorf((n + 1.0f) * 0.5f);

    CORDIC->CSR = CordicFunc::COSINE << CORDIC_CSR_FUNC_Pos
                  | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
                  | CORDIC_CSR_NRES;
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(n));

    cordic_poll();
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

    const float m = fmaxf(fabsf(x), fabsf(y));
    const float inv = 1.0f / m;

    CORDIC->CSR = CordicFunc::PHASE << CORDIC_CSR_FUNC_Pos
                  | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
                  | CORDIC_CSR_NARGS;
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(x * inv));
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(y * inv));

    cordic_poll();

    return from_q31(static_cast<int32_t>(CORDIC->RDATA)) * PI_F;
}

void cordic_atan2_mod(const float y, const float x, float &angle_out, float &mod_out) {
    if (x == 0.0f && y == 0.0f) {
        angle_out = 0.0f;
        mod_out = 0.0f;
        return;
    }

    const float m = fmaxf(fabsf(x), fabsf(y));
    const float inv = 1.0f / m;

    CORDIC->CSR = CordicFunc::PHASE << CORDIC_CSR_FUNC_Pos
                  | (DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos)
                  | CORDIC_CSR_NARGS | CORDIC_CSR_NRES;
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(x * inv));
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(y * inv));

    cordic_poll();

    angle_out = from_q31(static_cast<int32_t>(CORDIC->RDATA)) * PI_F;
    mod_out = from_uq31(CORDIC->RDATA) * m;
}

float cordic_modulus(const float y, const float x) {
    if (x == 0.0f && y == 0.0f) return 0.0f;

    const float m = fmaxf(fabsf(x), fabsf(y));
    const float inv = 1.0f / m;

    CORDIC->CSR = CordicFunc::MODULUS << CORDIC_CSR_FUNC_Pos
                  | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
                  | CORDIC_CSR_NARGS;
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(x * inv));
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(y * inv));

    cordic_poll();

    return from_uq31(CORDIC->RDATA) * m;
}

float cordic_atan(const float x) {
    CORDIC->CSR = CordicFunc::ARCTAN << CORDIC_CSR_FUNC_Pos
                  | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos;
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(x));

    cordic_poll();

    return from_q31(static_cast<int32_t>(CORDIC->RDATA)) * PI_F;
}

void cordic_sinh_cosh(const float x, float &sinh_out, float &cosh_out) {
    CORDIC->CSR = CordicFunc::HCOSINE << CORDIC_CSR_FUNC_Pos
                  | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
                  | CORDIC_CSR_NARGS | CORDIC_CSR_NRES;
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(x));
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(0.5f)); // m=0.5

    cordic_poll();
    
    cosh_out = from_q31(static_cast<int32_t>(CORDIC->RDATA)) * 2.0f;
    sinh_out = from_q31(static_cast<int32_t>(CORDIC->RDATA)) * 2.0f;
}

float cordic_sinh(const float x) {
    float s, c;
    cordic_sinh_cosh(x, s, c);
    return s;
}

float cordic_cosh(const float x) {
    float s, c;
    cordic_sinh_cosh(x, s, c);
    return c;
}

float cordic_atanh(const float x) {
    CORDIC->CSR = (CordicFunc::HATANH << CORDIC_CSR_FUNC_Pos)
                  | (DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos)
                  | CORDIC_CSR_NARGS;
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(x));
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(0.25f)); // m=0.25

    cordic_poll();

    return from_q31(static_cast<int32_t>(CORDIC->RDATA)) * 4.0f;
}

float cordic_ln(const float x) {
    if (x <= 0.0f) return -__builtin_inff();

    int exp;
    const float frac = frexpf(x, &exp);

    CORDIC->CSR = CordicFunc::LN << CORDIC_CSR_FUNC_Pos
                  | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
                  | 1U << CORDIC_CSR_SCALE_Pos;
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(frac));

    cordic_poll();

    const float ln_frac2 = from_q31(static_cast<int32_t>(CORDIC->RDATA));
    return ln_frac2 + static_cast<float>(exp - 1) * LN2;
}

float cordic_sqrt(const float x) {
    if (x <= 0.0f) return 0.0f;

    int exp;
    float frac = frexpf(x, &exp);

    if (exp & 1) {
        frac *= 0.5f;
        exp++;
    }
    if (frac >= 0.75f) {
        frac *= 0.25f;
        exp += 2;
    }

    CORDIC->CSR = CordicFunc::SQRT << CORDIC_CSR_FUNC_Pos
                  | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos;
    CORDIC->WDATA = static_cast<uint32_t>(to_q31(frac));

    cordic_poll();

    const float result = from_q31(static_cast<int32_t>(CORDIC->RDATA));
    return ldexpf(result, exp / 2);
}
