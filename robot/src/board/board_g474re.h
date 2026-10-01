#ifndef BUCKY_BOARD_G474RE_H
#define BUCKY_BOARD_G474RE_H

#include "hardware/io/i2c/I2CDMA.h"
#include "hardware/io/encoder/Encoder.h"
#include "hardware/motor/MotorDriver.h"
#include "hardware/sensors/pos/Sonar.h"

namespace board {

constexpr const char* NAME = "G474RE";

// ---- Motors (HRTIM1 A/C/F) ----
constexpr MotorPin MOTOR1 = {PA_8, PA_9};
constexpr MotorPin MOTOR2 = {PB_12, PB_13};
constexpr MotorPin MOTOR3 = {PC_7, PC_6};

// ---- Encoders (no timer pairs on these pads: EXTI backend) ----
constexpr EncoderPins ENCODERS[3] = {
    {PA_7, PA_6},   // M1
    {PB_2, PB_1},   // M2
    {PA_5, PA_2},   // M3
};

// ---- Sonar ----
constexpr SonarPins SONAR = {.trigPin = PB_11, .echoPins = {PA_10, PC_10, PC_11, PC_12}};

// ---- I2C (compass + accelerometer) ----
struct I2CPort {
    I2C_TypeDef* instance;
    PinName sda;
    PinName scl;
    DmaChannel* dmaRx;
    uint32_t dmaRxRequest;
};

inline const I2CPort SENSOR_I2C = {I2C1, PB_9, PA_15, DMA1_Channel6, DMA_REQUEST_I2C1_RX};
constexpr auto SENSOR_I2C_FREQ = I2CFrequency::FMP_3M4;

} // namespace board

#endif // BUCKY_BOARD_G474RE_H
