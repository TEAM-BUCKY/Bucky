#ifndef BUCKY_IRSENSOR_H
#define BUCKY_IRSENSOR_H

#include <Arduino.h>
#include <stm32g4xx.h>

enum class IRBallMode : uint8_t { MODE_D, MODE_A };

// Select IR ball protocol at compile time:
//   MODE_D: simple on/off burst — 1 sweep per cycle, distance = on/off only
//   MODE_A: stepped waveform (full/1/4/1/16/1/64) — 8 sweeps per cycle,
//           enables distance estimation from intensity steps
static constexpr auto IR_MODE = IRBallMode::MODE_A;

static constexpr uint32_t IR_MUX_CHANNELS = 16;

static constexpr bool IR_BOARD1_ENABLED = true;
static constexpr bool IR_BOARD2_ENABLED = true;

// Number of IR sensors physically connected per board (1-16).
static constexpr uint32_t IR_BOARD1_SENSOR_COUNT = 16;
static constexpr uint32_t IR_BOARD2_SENSOR_COUNT = 16;
static_assert(IR_BOARD1_SENSOR_COUNT >= 1 && IR_BOARD1_SENSOR_COUNT <= 16);
static_assert(IR_BOARD2_SENSOR_COUNT >= 1 && IR_BOARD2_SENSOR_COUNT <= 16);

static constexpr uint32_t IR_SWEEPS_PER_CYCLE =
    IR_MODE == IRBallMode::MODE_D ? 1 : 8;

static constexpr uint32_t IR_ADC_BUFFER_SIZE =
    IR_SWEEPS_PER_CYCLE * IR_MUX_CHANNELS;

void ir_sensor_init();

volatile uint16_t* ir_get_buffer(uint8_t board);
uint32_t ir_get_sensor_count(uint8_t board);

#endif // BUCKY_IRSENSOR_H
