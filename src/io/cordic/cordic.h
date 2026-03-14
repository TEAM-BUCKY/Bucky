#ifndef BUCKY_CORDIC_H
#define BUCKY_CORDIC_H

#include <stm32g4xx.h>

#include "optimizations/optimizations.h"

static constexpr float PI_F = 3.14159265358979f;
static constexpr float INV_PI = 1.0f / PI_F;

namespace CordicFunc {
    constexpr uint32_t COSINE  = 0;
    constexpr uint32_t SINE    = 1;
    constexpr uint32_t PHASE   = 2;  // atan2
    constexpr uint32_t MODULUS = 3;
    constexpr uint32_t ARCTAN  = 4;
    constexpr uint32_t HCOSINE = 5;
    constexpr uint32_t HSINE   = 6;
    constexpr uint32_t HATANH  = 7;
    constexpr uint32_t LN      = 8;
    constexpr uint32_t SQRT    = 9;
}

void FORCE_INLINE cordicInit() {
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
}

static FORCE_INLINE int32_t cordicCompute(uint32_t csr, int32_t arg)
{
    int32_t result;
    asm volatile(
        "STR %[csr], [%[base], %[csr_off]]\n\t"
        "STR %[arg], [%[base], %[wd_off]]\n\t"
        "1: LDR %[res], [%[base], %[csr_off]]\n\t"
        "TST %[res], %[rdy]\n\t"
        "BEQ 1b\n\t"
        "LDR %[res], [%[base], %[rd_off]]"
        : [res] "=&r" (result)
        : [base] "r" (CORDIC),
        [csr] "r" (csr), [arg] "r" (arg),
        [rdy] "I" (CORDIC_CSR_RRDY),
        [csr_off] "J" (offsetof(CORDIC_TypeDef, CSR)),
        [wd_off] "J" (offsetof(CORDIC_TypeDef, WDATA)),
        [rd_off] "J" (offsetof(CORDIC_TypeDef, RDATA))
        : "memory"
    );
    return result;
}

static FORCE_INLINE int32_t cordicCompute2(uint32_t csr, int32_t arg1, int32_t arg2)
{
    int32_t result;
    asm volatile(
        "STR %[csr], [%[base], %[csr_off]]\n\t"
        "STR %[a1],  [%[base], %[wd_off]]\n\t"
        "STR %[a2],  [%[base], %[wd_off]]\n\t"
        "1: LDR %[res], [%[base], %[csr_off]]\n\t"
        "TST %[res], %[rdy]\n\t"
        "BEQ 1b\n\t"
        "LDR %[res], [%[base], %[rd_off]]"
        : [res] "=&r" (result)
        : [base] "r" (CORDIC),
          [csr] "r" (csr), [a1] "r" (arg1), [a2] "r" (arg2),
          [rdy] "I" (CORDIC_CSR_RRDY),
          [csr_off] "J" (offsetof(CORDIC_TypeDef, CSR)),
          [wd_off] "J" (offsetof(CORDIC_TypeDef, WDATA)),
          [rd_off] "J" (offsetof(CORDIC_TypeDef, RDATA))
        : "memory"
    );
    return result;
}

static FORCE_INLINE void cordicComputeRes2(uint32_t csr, int32_t arg, int32_t &res1, int32_t &res2)
{
    asm volatile(
        "STR %[csr], [%[base], %[csr_off]]\n\t"
        "STR %[arg], [%[base], %[wd_off]]\n\t"
        "1: LDR %[r1], [%[base], %[csr_off]]\n\t"
        "TST %[r1], %[rdy]\n\t"
        "BEQ 1b\n\t"
        "LDR %[r1], [%[base], %[rd_off]]\n\t"
        "LDR %[r2], [%[base], %[rd_off]]"
        : [r1] "=&r" (res1), [r2] "=&r" (res2)
        : [base] "r" (CORDIC),
          [csr] "r" (csr), [arg] "r" (arg),
          [rdy] "I" (CORDIC_CSR_RRDY),
          [csr_off] "J" (offsetof(CORDIC_TypeDef, CSR)),
          [wd_off] "J" (offsetof(CORDIC_TypeDef, WDATA)),
          [rd_off] "J" (offsetof(CORDIC_TypeDef, RDATA))
        : "memory"
    );
}

static FORCE_INLINE void cordicCompute2Res2(uint32_t csr, int32_t arg1, int32_t arg2, int32_t &res1, int32_t &res2)
{
    asm volatile(
        "STR %[csr], [%[base], %[csr_off]]\n\t"
        "STR %[a1],  [%[base], %[wd_off]]\n\t"
        "STR %[a2],  [%[base], %[wd_off]]\n\t"
        "1: LDR %[r1], [%[base], %[csr_off]]\n\t"
        "TST %[r1], %[rdy]\n\t"
        "BEQ 1b\n\t"
        "LDR %[r1], [%[base], %[rd_off]]\n\t"
        "LDR %[r2], [%[base], %[rd_off]]"
        : [r1] "=&r" (res1), [r2] "=&r" (res2)
        : [base] "r" (CORDIC),
          [csr] "r" (csr), [a1] "r" (arg1), [a2] "r" (arg2),
          [rdy] "I" (CORDIC_CSR_RRDY),
          [csr_off] "J" (offsetof(CORDIC_TypeDef, CSR)),
          [wd_off] "J" (offsetof(CORDIC_TypeDef, WDATA)),
          [rd_off] "J" (offsetof(CORDIC_TypeDef, RDATA))
        : "memory"
    );
}

void cordicSinCos(float angle_rad, float& sin_out, float& cos_out);
float cordicSin(float angle_rad);
float cordicCos(float angle_rad);
float cordicAtan2(float y, float x);
void cordicAtan2Mod(float y, float x, float& angle_out, float& mod_out);
float cordicModulus(float y, float x);
float cordicAan(float x);

void cordicSinhCosh(float x, float& sinh_out, float& cosh_out);
float cordicSinh(float x);
float cordicCosh(float x);
float cordicAtanh(float x);

float cordicLn(float x);
float cordicSqrt(float x);

#endif // BUCKY_CORDIC_H
