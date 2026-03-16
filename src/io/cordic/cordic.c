#include "cordic.h"
#include <math.h>

#include "optimizations/optimizations.h"

#define Q31_SCALE 2147483648.0f
#define Q31_INV   (1.0f / Q31_SCALE)
#define LN2_F     0.69314718f

#define DEFAULT_PRECISION 6

static FORCE_INLINE int32_t to_q31(float x)
{
    return x >= 1.0f ? 0x7FFFFFFF : (int32_t)(x * Q31_SCALE);
}

static FORCE_INLINE float from_q31(int32_t x)
{
    float result;
    asm volatile(
        "VMOV %[res], %[x]\n\t"
        "VCVT.F32.S32 %[res], %[res], #31"
        : [res] "=t" (result)
        : [x] "r" (x)
    );
    return result;
}

static FORCE_INLINE float from_uq31(uint32_t x)
{
    float result;
    asm volatile(
        "VMOV %[res], %[x]\n\t"
        "VCVT.F32.U32 %[res], %[res], #31"
        : [res] "=t" (result)
        : [x] "r" (x)
    );
    return result;
}

void cordic_sin_cos(float angle_rad, float *sin_out, float *cos_out)
{
    float n = angle_rad * INV_PI_F;
    n -= 2.0f * floorf((n + 1.0f) * 0.5f);

    uint32_t csr = CORDIC_FUNC_COSINE << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | CORDIC_CSR_NRES;
    int32_t r1, r2;
    cordic_compute_res2(csr, to_q31(n), &r1, &r2);
    *cos_out = from_q31(r1);
    *sin_out = from_q31(r2);
}

float cordic_sin(float angle_rad)
{
    float s, c;
    cordic_sin_cos(angle_rad, &s, &c);
    return s;
}

float cordic_cos(float angle_rad)
{
    float s, c;
    cordic_sin_cos(angle_rad, &s, &c);
    return c;
}

float cordic_atan2(float y, float x)
{
    if (x == 0.0f && y == 0.0f) return 0.0f;

    float m = fmaxf(fabsf(x), fabsf(y));
    float inv = 1.0f / m;

    uint32_t csr = CORDIC_FUNC_PHASE << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | CORDIC_CSR_NARGS;

    return from_q31(cordic_compute2(csr, to_q31(x * inv), to_q31(y * inv))) * PI_F;
}

void cordic_atan2_mod(float y, float x, float *angle_out, float *mod_out)
{
    if (x == 0.0f && y == 0.0f)
    {
        *angle_out = 0.0f;
        *mod_out = 0.0f;
        return;
    }

    float m = fmaxf(fabsf(x), fabsf(y));
    float inv = 1.0f / m;

    uint32_t csr = CORDIC_FUNC_PHASE << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | CORDIC_CSR_NARGS | CORDIC_CSR_NRES;
    int32_t r1, r2;
    cordic_compute2_res2(csr, to_q31(x * inv), to_q31(y * inv), &r1, &r2);
    *angle_out = from_q31(r1) * PI_F;
    *mod_out = from_uq31((uint32_t)r2) * m;
}

float cordic_modulus(float y, float x)
{
    if (x == 0.0f && y == 0.0f) return 0.0f;

    float m = fmaxf(fabsf(x), fabsf(y));
    float inv = 1.0f / m;

    uint32_t csr = CORDIC_FUNC_MODULUS << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | CORDIC_CSR_NARGS;

    return from_uq31((uint32_t)cordic_compute2(csr, to_q31(x * inv), to_q31(y * inv))) * m;
}

float cordic_atan(float x)
{
    uint32_t csr = CORDIC_FUNC_ARCTAN << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos;

    return from_q31(cordic_compute(csr, to_q31(x))) * PI_F;
}

void cordic_sinh_cosh(float x, float *sinh_out, float *cosh_out)
{
    uint32_t csr = CORDIC_FUNC_HCOSINE << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | CORDIC_CSR_NARGS | CORDIC_CSR_NRES;
    int32_t r1, r2;
    cordic_compute2_res2(csr, to_q31(x), to_q31(0.5f), &r1, &r2);
    *cosh_out = from_q31(r1) * 2.0f;
    *sinh_out = from_q31(r2) * 2.0f;
}

float cordic_sinh(float x)
{
    float s, c;
    cordic_sinh_cosh(x, &s, &c);
    return s;
}

float cordic_cosh(float x)
{
    float s, c;
    cordic_sinh_cosh(x, &s, &c);
    return c;
}

float cordic_atanh(float x)
{
    uint32_t csr = CORDIC_FUNC_HATANH << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | CORDIC_CSR_NARGS;

    return from_q31(cordic_compute2(csr, to_q31(x), to_q31(0.25f))) * 4.0f;
}

float cordic_ln(float x)
{
    if (x <= 0.0f) return -__builtin_inff();

    int exp;
    float frac = frexpf(x, &exp);

    uint32_t csr = CORDIC_FUNC_LN << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos
        | 1U << CORDIC_CSR_SCALE_Pos;

    float ln_frac2 = from_q31(cordic_compute(csr, to_q31(frac)));
    return ln_frac2 + (float)(exp - 1) * LN2_F;
}

float cordic_sqrt(float x)
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

    uint32_t csr = CORDIC_FUNC_SQRT << CORDIC_CSR_FUNC_Pos
        | DEFAULT_PRECISION << CORDIC_CSR_PRECISION_Pos;

    return ldexpf(from_q31(cordic_compute(csr, to_q31(frac))), exp / 2);
}
