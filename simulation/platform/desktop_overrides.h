#ifndef BUCKY_DESKTOP_OVERRIDES_H
#define BUCKY_DESKTOP_OVERRIDES_H

// =============================================================================
// Desktop overrides for Bucky robot code
// =============================================================================
//
// This header is force-included (-include) before every translation unit.
// It pre-defines include guards for hardware-specific headers so that when
// robot algorithm code #includes them, the real headers are never processed.
// Desktop-compatible replacements are provided inline below.
//
// This means ZERO modifications to the Bucky source code are needed.
// =============================================================================

#include <cstdint>
#include <cmath>

// -----------------------------------------------------------------------------
// Block: sensors/IRSensor.h  (needs <Arduino.h>, <stm32g4xx.h>)
// Provide only the constants and types that algorithm code needs.
// -----------------------------------------------------------------------------
#define BUCKY_IRSENSOR_H

enum class IRBallMode : uint8_t { MODE_D, MODE_A };

static constexpr auto IR_MODE = IRBallMode::MODE_A;
static constexpr uint32_t IR_MUX_CHANNELS = 16;
static constexpr bool IR_BOARD1_ENABLED = true;
static constexpr bool IR_BOARD2_ENABLED = true;
static constexpr uint32_t IR_BOARD1_SENSOR_COUNT = 16;
static constexpr uint32_t IR_BOARD2_SENSOR_COUNT = 16;
static constexpr uint32_t IR_SWEEPS_PER_CYCLE =
    IR_MODE == IRBallMode::MODE_D ? 1 : 8;
static constexpr uint32_t IR_ADC_BUFFER_SIZE =
    IR_SWEEPS_PER_CYCLE * IR_MUX_CHANNELS;

// Stub declarations (never called in simulation, but needed for linkage)
inline void ir_sensor_init() {}
inline const uint16_t* ir_get_buffer(uint8_t) { return nullptr; }
inline uint32_t ir_get_sensor_count(uint8_t) { return 16; }
inline uint32_t ir_get_frame_sequence(uint8_t) { return 0; }
inline bool ir_has_new_frame(uint8_t, uint32_t) { return false; }

// -----------------------------------------------------------------------------
// Block: motor/MotorDriver.h  (needs <Arduino.h>, io/gpio/pwm.h)
// Provide only VectorXY and minimal types used by StrategyFSM.
// -----------------------------------------------------------------------------
#define BUCKY_MOTORDRIVER_H

#define MIN_SPEED 1700
#define MAX_SPEED 3399

struct MotorPin {
    int inA;
    int inB;
};

struct VectorXY {
    float x;
    float y;
};

// -----------------------------------------------------------------------------
// Block: io/cordic/cordic.h  (ARM inline ASM + STM32 CORDIC coprocessor)
// Provide desktop implementations using <cmath>.
// -----------------------------------------------------------------------------
#define BUCKY_CORDIC_H

#ifdef __cplusplus
extern "C" {
#endif

static inline void cordic_init() {}

static inline void cordic_sin_cos(float angle_rad, float* sin_out, float* cos_out)
{
    *sin_out = sinf(angle_rad);
    *cos_out = cosf(angle_rad);
}

static inline float cordic_sin(float angle_rad) { return sinf(angle_rad); }
static inline float cordic_cos(float angle_rad) { return cosf(angle_rad); }

static inline float cordic_atan2(float y, float x) { return atan2f(y, x); }

static inline void cordic_atan2_mod(float y, float x, float* angle_out, float* mod_out)
{
    *angle_out = atan2f(y, x);
    *mod_out = hypotf(y, x);
}

static inline float cordic_modulus(float y, float x) { return hypotf(y, x); }
static inline float cordic_atan(float x) { return atanf(x); }

static inline void cordic_sinh_cosh(float x, float* sinh_out, float* cosh_out)
{
    *sinh_out = sinhf(x);
    *cosh_out = coshf(x);
}

static inline float cordic_sinh(float x) { return sinhf(x); }
static inline float cordic_cosh(float x) { return coshf(x); }
static inline float cordic_atanh(float x) { return atanhf(x); }
static inline float cordic_ln(float x) { return logf(x); }
static inline float cordic_sqrt(float x) { return sqrtf(x); }

#ifdef __cplusplus
}
#endif

// -----------------------------------------------------------------------------
// Sonar count constant (from sensors/pos/Sonar.h)
// -----------------------------------------------------------------------------
#ifndef SONAR_COUNT
#define SONAR_COUNT 4
#endif

#endif // BUCKY_DESKTOP_OVERRIDES_H
