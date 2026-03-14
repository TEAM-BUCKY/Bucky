#include "cordic.h"
#include "../bitboard/bitboard.h"
#include <cmath>

#include "optimizations/optimizations.h"

static constexpr float Q31_SCALE = 2147483648.0f;
static constexpr float Q31_INV = 1.0f / Q31_SCALE;
static constexpr float LN2 = 0.69314718f;

static constexpr uint32_t DEFAULT_PRECISION = 6;

static FORCE_INLINE int32_t to_q31(const float x)
{
    return x >= 1.0f ? 0x7FFFFFFF : static_cast<int32_t>(x * Q31_SCALE);
}

static FORCE_INLINE float from_q31(const int32_t x)
{
    return static_cast<float>(x) * Q31_INV;
}

static FORCE_INLINE float from_uq31(const uint32_t x)
{
    return static_cast<float>(x) * Q31_INV;
}

void cordicSinCos(const float angle_rad, float& sin_out, float& cos_out)
{
    float n = angle_rad * INV_PI;
    n -= 2.0f * floorf((n + 1.0f) * 0.5f);

    constexpr uint32_t csr = CordicFunc::COSINE << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | CORDIC_CSR_NRES;
    int32_t r1, r2;
    cordicComputeRes2(csr, to_q31(n), r1, r2);
    cos_out = from_q31(r1);
    sin_out = from_q31(r2);
}

float cordicSin(const float angle_rad)
{
    float s, c;
    cordicSinCos(angle_rad, s, c);
    return s;
}

float cordicCos(const float angle_rad)
{
    float s, c;
    cordicSinCos(angle_rad, s, c);
    return c;
}

float cordicAtan2(const float y, const float x)
{
    if (x == 0.0f && y == 0.0f) return 0.0f;

    const float m = fmaxf(fabsf(x), fabsf(y));
    const float inv = 1.0f / m;

    constexpr uint32_t csr = CordicFunc::PHASE << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | CORDIC_CSR_NARGS;

    return from_q31(cordicCompute2(csr, to_q31(x * inv), to_q31(y * inv))) * PI_F;
}

void cordicAtan2Mod(const float y, const float x, float& angle_out, float& mod_out)
{
    if (x == 0.0f && y == 0.0f)
    {
        angle_out = 0.0f;
        mod_out = 0.0f;
        return;
    }

    const float m = fmaxf(fabsf(x), fabsf(y));
    const float inv = 1.0f / m;

    constexpr uint32_t csr = CordicFunc::PHASE << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | CORDIC_CSR_NARGS | CORDIC_CSR_NRES;
    int32_t r1, r2;
    cordicCompute2Res2(csr, to_q31(x * inv), to_q31(y * inv), r1, r2);
    angle_out = from_q31(r1) * PI_F;
    mod_out = from_uq31(static_cast<uint32_t>(r2)) * m;
}

float cordicModulus(const float y, const float x)
{
    if (x == 0.0f && y == 0.0f) return 0.0f;

    const float m = fmaxf(fabsf(x), fabsf(y));
    const float inv = 1.0f / m;

    constexpr uint32_t csr = CordicFunc::MODULUS << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | CORDIC_CSR_NARGS;

    return from_uq31(static_cast<uint32_t>(cordicCompute2(csr, to_q31(x * inv), to_q31(y * inv)))) * m;
}

float cordicAan(const float x)
{
    constexpr uint32_t csr = CordicFunc::ARCTAN << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos;

    return from_q31(cordicCompute(csr, to_q31(x))) * PI_F;
}

void cordicSinhCosh(const float x, float& sinh_out, float& cosh_out)
{
    constexpr uint32_t csr = CordicFunc::HCOSINE << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | CORDIC_CSR_NARGS | CORDIC_CSR_NRES;
    int32_t r1, r2;
    cordicCompute2Res2(csr, to_q31(x), to_q31(0.5f), r1, r2);
    cosh_out = from_q31(r1) * 2.0f;
    sinh_out = from_q31(r2) * 2.0f;
}

float cordicSinh(const float x)
{
    float s, c;
    cordicSinhCosh(x, s, c);
    return s;
}

float cordicCosh(const float x)
{
    float s, c;
    cordicSinhCosh(x, s, c);
    return c;
}

float cordic_atanh(const float x)
{
    constexpr uint32_t csr = CordicFunc::HATANH << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | CORDIC_CSR_NARGS;

    return from_q31(cordicCompute2(csr, to_q31(x), to_q31(0.25f))) * 4.0f;
}

float cordicLn(const float x)
{
    if (x <= 0.0f) return -__builtin_inff();

    int exp;
    const float frac = frexpf(x, &exp);

    constexpr uint32_t csr = CordicFunc::LN << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | 1U << CORDIC_CSR_SCALE_Pos;

    const float ln_frac2 = from_q31(cordicCompute(csr, to_q31(frac)));
    return ln_frac2 + static_cast<float>(exp - 1) * LN2;
}

float cordicSqrt(const float x)
{
    if (x <= 0.0f) return 0.0f;

    int exp;
    float frac = frexpf(x, &exp);

    if (exp & 1)
    {
        frac *= 0.5f;
        exp++;
    }
    if (frac >= 0.75f)
    {
        frac *= 0.25f;
        exp += 2;
    }

    constexpr uint32_t csr = CordicFunc::SQRT << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos;

    return ldexpf(from_q31(cordicCompute(csr, to_q31(frac))), exp / 2);
}
