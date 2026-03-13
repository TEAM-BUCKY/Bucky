#ifndef BUCKY_CORDIC_H
#define BUCKY_CORDIC_H

#include <stm32g4xx.h>

namespace CordicFunc {
    constexpr uint32_t COSINE  = 0;
    constexpr uint32_t SINE    = 1;
    constexpr uint32_t PHASE   = 2;  // atan2
    constexpr uint32_t MODULUS = 3;
    constexpr uint32_t ARCTAN  = 4;
    constexpr uint32_t SQRT    = 9;
}

void cordic_init();
void cordic_sin_cos(float angle_rad, float& sin_out, float& cos_out);
float cordic_sin(float angle_rad);
float cordic_cos(float angle_rad);
float cordic_atan2(float y, float x);

#endif // BUCKY_CORDIC_H
