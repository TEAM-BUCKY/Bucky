#ifndef BUCKY_IRSENSOR_H
#define BUCKY_IRSENSOR_H

#include <Arduino.h>
#include <stm32g4xx.h>

#include "optimizations/optimizations.h"

enum class IRBallMode : uint8_t { MODE_D, MODE_A };

// Select IR ball protocol at compile time:
//   MODE_D: simple on/off burst — 1 sweep per cycle, distance = on/off only
//   MODE_A: stepped waveform (full/1/4/1/16/1/64) — 8 sweeps per cycle,
//           enables distance estimation from intensity steps
static constexpr auto IR_MODE = IRBallMode::MODE_A;

static constexpr uint32_t IR_MUX_CHANNELS = 16;

static constexpr bool IR_BOARD1_ENABLED = false;
static constexpr bool IR_BOARD2_ENABLED = true;

// Number of IR sensors physically connected per board (1-16).
static constexpr uint32_t IR_BOARD1_SENSOR_COUNT = 16;
static constexpr uint32_t IR_BOARD2_SENSOR_COUNT = 12;
static_assert(IR_BOARD1_SENSOR_COUNT >= 1 && IR_BOARD1_SENSOR_COUNT <= 16);
static_assert(IR_BOARD2_SENSOR_COUNT >= 1 && IR_BOARD2_SENSOR_COUNT <= 16);

static constexpr uint32_t IR_SWEEPS_PER_CYCLE =
    IR_MODE == IRBallMode::MODE_D ? 1 : 16;

static constexpr uint32_t IR_ADC_BUFFER_SIZE =
    IR_SWEEPS_PER_CYCLE * IR_MUX_CHANNELS;

void ir_sensor_init();

const uint16_t* ir_get_buffer(uint8_t board);
uint32_t ir_get_sensor_count(uint8_t board);

uint32_t ir_get_frame_sequence(uint8_t board);
bool ir_has_new_frame(uint8_t board, uint32_t lastSequence);

// Mux channel rotation calibration. The 74HC4040 counter driving the mux is
// free-running and its state at the first gate cycle is unpredictable, so
// buffer index 0 doesn't reliably map to physical sensor 0 across boots.
// We have 16 mux channels but only IR_BOARDx_SENSOR_COUNT connected sensors,
// so the unconnected channels always read ~0. Calibration finds that
// cluster and derives the offset such that physical sensor i lives at
// buffer index (i + offset) % 16.
//
// Call after ir_sensor_init() with the field clear of IR sources. Returns
// the detected offset and stores it for later retrieval via
// ir_get_channel_offset().
uint8_t ir_calibrate_channels(uint8_t board, uint32_t frames = 30);
uint8_t ir_get_channel_offset(uint8_t board);

#endif // BUCKY_IRSENSOR_H
