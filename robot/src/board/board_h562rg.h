#ifndef BUCKY_BOARD_H562RG_H
#define BUCKY_BOARD_H562RG_H

#include "hardware/io/i2c/I2CDMA.h"
#include "hardware/io/uart/UARTDMA.h"
#include "hardware/io/encoder/Encoder.h"
#include "hardware/motor/Motors.h"
#include "hardware/sensors/GPort.h"
#include "hardware/sensors/pos/Sonar.h"

#define BOARD_HAS_BLUETOOTH 1
#define BOARD_HAS_GPORTS 1
#define BOARD_HAS_BUTTONS 1
#define BOARD_HAS_KICKER 1

namespace Board {

constexpr auto NAME = "H562RG";

constexpr MotorPin MOTOR1 = {PB_14_ALT2, PB_15_ALT2};   // TIM12_CH1 / TIM12_CH2
constexpr MotorPin MOTOR2 = {PA_9, PA_10};              // TIM1_CH2  / TIM1_CH3
constexpr MotorPin MOTOR3 = {PB_6, PC_2};               // TIM4_CH1  / TIM4_CH4

constexpr PinName BUTTON1 = PB_0;
constexpr PinName BUTTON2 = PB_1;

constexpr EncoderPins ENCODERS[3] = {
    {PB_4, PB_5},             // M1: TIM3_CH1 / TIM3_CH2
    {PA_0, PA_1},             // M2: TIM2_CH1 / TIM2_CH2
    {PC_6_ALT1, PC_7_ALT1},   // M3: TIM8_CH1 / TIM8_CH2
};

constexpr SonarPins SONAR = {.trigPin = PC_8, .echoPins = {PC_10, PC_11, PA_14, PD_2}};

constexpr PinName KICKER = PA_6_ALT1;   // TIM13_CH1

constexpr PinName ANALOG[4] = {PA_4, PA_5, PC_4, PC_5};   // INP18, INP19, INP4, INP8

constexpr uint8_t PB2_LPTIM1_CH1_AF = 5;   // AF1 on PB2 is not LPTIM1; see datasheet AF table

inline const GPortHardware G_PORT1 = {
    .resetPin = PA_15, .resetEdge = EXTI_RISING,
    .clockPin = PC_12, .clockLptim = nullptr, .clockLptimChannel = 0, .clockLptimAf = 0,
    .adcTrigger = ADC_EXTERNALTRIG_T15_TRGO, .muxAdvanceEdge = GClockEdge::Falling,
    .adcPin = PC_3,                                   // ADC1_INP13
    .dma = GPDMA1_Channel4, .dmaRequest = GPDMA1_REQUEST_ADC1,
    .modulationPin = PB_7_ALT1,                       // TIM17_CH1N
};

inline const GPortHardware G_PORT2 = {
    .resetPin = PB_12, .resetEdge = EXTI_RISING,
    .clockPin = PB_2, .clockLptim = LPTIM1, .clockLptimChannel = 0, .clockLptimAf = PB2_LPTIM1_CH1_AF,
    .adcTrigger = ADC_EXTERNALTRIG_LPTIM1_CH1, .muxAdvanceEdge = GClockEdge::Falling,
    .adcPin = PC_0_ALT1,                              // ADC2_INP10
    .dma = GPDMA1_Channel5, .dmaRequest = GPDMA1_REQUEST_ADC2,
    .modulationPin = PB_8_ALT1,                       // TIM16_CH1
};

constexpr auto G_PORT1_KIND = GSensorKind::IR;
constexpr auto G_PORT2_KIND = GSensorKind::Line;

struct I2CPort {
    I2C_TypeDef* instance;
    PinName sda;
    PinName scl;
    DmaChannel* dmaRx;
    uint32_t dmaRxRequest;
};

inline const I2CPort I2C_2 = {I2C2, PB_3, PB_10, GPDMA1_Channel0, GPDMA1_REQUEST_I2C2_RX};
inline const I2CPort I2C_3 = {I2C3, PC_9, PA_8,  GPDMA1_Channel1, GPDMA1_REQUEST_I2C3_RX};

inline const I2CPort& SENSOR_I2C = I2C_2;
constexpr auto SENSOR_I2C_FREQ = I2CFrequency::FM_400K;

inline const UartHardware BLUETOOTH_UART = {USART2, PA_2, PA_3,
                                            GPDMA1_Channel3, GPDMA1_REQUEST_USART2_TX,
                                            GPDMA1_Channel2, GPDMA1_REQUEST_USART2_RX};
constexpr uint32_t BLUETOOTH_BAUD = 115200;   // TODO: match the module's configured rate

}

#endif // BUCKY_BOARD_H562RG_H
