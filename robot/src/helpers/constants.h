#ifndef BUCKY_CONSTANTS_H
#define BUCKY_CONSTANTS_H

// Also included from C (cordic.c), which has no constexpr.
#ifdef __cplusplus
constexpr float PI_F = 3.14159265358979f;
constexpr float INV_PI_F = 0.3183098861837907f;
constexpr float SIN_60_F = 0.866025403784439f;
#else
#define PI_F     3.14159265358979f
#define INV_PI_F 0.3183098861837907f
#define SIN_60_F 0.866025403784439f
#endif

#endif //BUCKY_CONSTANTS_H