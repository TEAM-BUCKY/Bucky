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

void FORCE_INLINE cordic_init() {
    asm("ORR %[mask], %[reg]" : [reg] "+m" (RCC->AHB1ENR) : [mask] "r" (RCC_AHB1ENR_CORDICEN));
}

void cordic_sin_cos(float angle_rad, float& sin_out, float& cos_out);
float cordic_sin(float angle_rad);
float cordic_cos(float angle_rad);
float cordic_atan2(float y, float x);
void cordic_atan2_mod(float y, float x, float& angle_out, float& mod_out);
float cordic_modulus(float y, float x);
float cordic_atan(float x);

void cordic_sinh_cosh(float x, float& sinh_out, float& cosh_out);
float cordic_sinh(float x);
float cordic_cosh(float x);
float cordic_atanh(float x);

float cordic_ln(float x);
float cordic_sqrt(float x);

#endif // BUCKY_CORDIC_H
