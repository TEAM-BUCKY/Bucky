#ifndef BUCKY_CORDIC_H
#define BUCKY_CORDIC_H

#include <stm32g4xx.h>

#include "optimizations/optimizations.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PI_F      3.14159265358979f
#define INV_PI_F  (1.0f / PI_F)

#define CORDIC_FUNC_COSINE  0
#define CORDIC_FUNC_SINE    1
#define CORDIC_FUNC_PHASE   2
#define CORDIC_FUNC_MODULUS 3
#define CORDIC_FUNC_ARCTAN  4
#define CORDIC_FUNC_HCOSINE 5
#define CORDIC_FUNC_HSINE   6
#define CORDIC_FUNC_HATANH  7
#define CORDIC_FUNC_LN      8
#define CORDIC_FUNC_SQRT    9

static FORCE_INLINE void cordic_init()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
}

static FORCE_INLINE int32_t cordic_compute(uint32_t csr, int32_t arg)
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

static FORCE_INLINE int32_t cordic_compute2(uint32_t csr, int32_t arg1, int32_t arg2)
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

static FORCE_INLINE void cordic_compute_res2(uint32_t csr, int32_t arg, int32_t *res1, int32_t *res2)
{
    asm volatile(
        "STR %[csr], [%[base], %[csr_off]]\n\t"
        "STR %[arg], [%[base], %[wd_off]]\n\t"
        "1: LDR %[r1], [%[base], %[csr_off]]\n\t"
        "TST %[r1], %[rdy]\n\t"
        "BEQ 1b\n\t"
        "LDR %[r1], [%[base], %[rd_off]]\n\t"
        "LDR %[r2], [%[base], %[rd_off]]"
        : [r1] "=&r" (*res1), [r2] "=&r" (*res2)
        : [base] "r" (CORDIC),
          [csr] "r" (csr), [arg] "r" (arg),
          [rdy] "I" (CORDIC_CSR_RRDY),
          [csr_off] "J" (offsetof(CORDIC_TypeDef, CSR)),
          [wd_off] "J" (offsetof(CORDIC_TypeDef, WDATA)),
          [rd_off] "J" (offsetof(CORDIC_TypeDef, RDATA))
        : "memory"
    );
}

static FORCE_INLINE void cordic_compute2_res2(uint32_t csr, int32_t arg1, int32_t arg2, int32_t *res1, int32_t *res2)
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
        : [r1] "=&r" (*res1), [r2] "=&r" (*res2)
        : [base] "r" (CORDIC),
          [csr] "r" (csr), [a1] "r" (arg1), [a2] "r" (arg2),
          [rdy] "I" (CORDIC_CSR_RRDY),
          [csr_off] "J" (offsetof(CORDIC_TypeDef, CSR)),
          [wd_off] "J" (offsetof(CORDIC_TypeDef, WDATA)),
          [rd_off] "J" (offsetof(CORDIC_TypeDef, RDATA))
        : "memory"
    );
}

void cordic_sin_cos(float angle_rad, float *sin_out, float *cos_out);
float cordic_sin(float angle_rad);
float cordic_cos(float angle_rad);
float cordic_atan2(float y, float x);
void cordic_atan2_mod(float y, float x, float *angle_out, float *mod_out);
float cordic_modulus(float y, float x);
float cordic_atan(float x);

void cordic_sinh_cosh(float x, float *sinh_out, float *cosh_out);
float cordic_sinh(float x);
float cordic_cosh(float x);
float cordic_atanh(float x);

float cordic_ln(float x);
float cordic_sqrt(float x);

#ifdef __cplusplus
}
#endif

#endif // BUCKY_CORDIC_H
