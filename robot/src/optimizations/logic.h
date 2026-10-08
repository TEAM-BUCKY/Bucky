#ifndef BUCKY_LOGIC_H
#define BUCKY_LOGIC_H

#include <cmath>

#include "optimizations.h"

static FORCE_INLINE float clamp(const float value, const float min, const float max)
{
    return fmaxf(min, fminf(max, value));
}

#endif //BUCKY_LOGIC_H