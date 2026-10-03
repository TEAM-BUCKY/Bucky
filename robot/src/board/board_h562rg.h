#ifndef BUCKY_BOARD_H562RG_H
#define BUCKY_BOARD_H562RG_H

#include "hardware/io/i2c/I2CDMA.h"
#include "hardware/io/uart/UARTDMA.h"
#include "hardware/io/encoder/Encoder.h"
#include "hardware/motor/MotorDriver.h"
#include "hardware/sensors/GPort.h"
#include "hardware/sensors/pos/Sonar.h"

#define BOARD_HAS_BLUETOOTH 1
#define BOARD_HAS_GPORTS 1
#define BOARD_HAS_BUTTONS 1

// _ALTn suffixes pick the timer when a pad has several (see PeripheralPins.c of
// the H562R(G-I)T variant). Without them the core's first match wins, e.g.
// PB_14 would resolve to TIM1_CH2N instead of TIM12_CH1.

namespace board {

constexpr const char* NAME = "H562RG";

// ---- Motors ----
constexpr MotorPin MOTOR1 = {PB_14_ALT2, PB_15_ALT2};   // TIM12_CH1 / TIM12_CH2
constexpr MotorPin MOTOR2 = {PA_9, PA_10};              // TIM1_CH2  / TIM1_CH3
constexpr MotorPin MOTOR3 = {PB_6, PC_2};               // TIM4_CH1  / TIM4_CH4

constexpr PinName BUTTON1 = PB_0;
constexpr PinName BUTTON2 = PB_1;

// ---- Encoders (hardware quadrature) ----
constexpr EncoderPins ENCODERS[3] = {
    {PB_4, PB_5},             // M1: TIM3_CH1 / TIM3_CH2
    {PA_0, PA_1},             // M2: TIM2_CH1 / TIM2_CH2
    {PC_6_ALT1, PC_7_ALT1},   // M3: TIM8_CH1 / TIM8_CH2
};

// ---- Sonar ----
// PA14 is SWCLK: once its echo interrupt is attached, SWD debugging is lost.
constexpr SonarPins SONAR = {.trigPin = PC_8, .echoPins = {PC_10, PC_11, PA_14, PD_2}};

// ---- Kicker ----
constexpr PinName KICKER_PWM = PA_6_ALT1;   // TIM13_CH1

// ---- Analog inputs (ADC12) ----
constexpr PinName ANALOG[4] = {PA_4, PA_5, PC_4, PC_5};   // INP18, INP19, INP4, INP8

// ---- Generalised sensor ports (IR ring or line sensor) ----
// Each port has its own ADC so both can be hardware-triggered by their clock:
//   G1: TIM15_CH1 -> TIM15_TRGO -> ADC1,  G2: LPTIM1_CH1 -> ADC2.
// Mux advance edge and reset polarity are assumptions until checked on the boards.

// TODO: confirm in the H562 datasheet alternate-function table (PB2 LPTIM1_CH1).
constexpr uint8_t PB2_LPTIM1_CH1_AF = 1;

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

// TODO: set to what is plugged into each port.
constexpr GSensorKind G_PORT1_KIND = GSensorKind::IR;
constexpr GSensorKind G_PORT2_KIND = GSensorKind::Line;

// ---- General purpose inputs ----
constexpr PinName GENERAL_PURPOSE[2] = {PH_0, PC_1};   // EXTI0, EXTI1

// ---- I2C ----
struct I2CPort {
    I2C_TypeDef* instance;
    PinName sda;
    PinName scl;
    DmaChannel* dmaRx;
    uint32_t dmaRxRequest;
};

inline const I2CPort I2C_2 = {I2C2, PB_3, PB_10, GPDMA1_Channel0, GPDMA1_REQUEST_I2C2_RX};
inline const I2CPort I2C_3 = {I2C3, PC_9, PA_8,  GPDMA1_Channel1, GPDMA1_REQUEST_I2C3_RX};

// TODO: confirm which bus the compass/accelerometer sit on.
inline const I2CPort& SENSOR_I2C = I2C_2;
// No FM+ pad drivers on PB3/PB10/PC9/PA8 (H5 only has them on PB6-PB9).
constexpr auto SENSOR_I2C_FREQ = I2CFrequency::FM_400K;

// ---- Bluetooth module (USART2) ----
inline const UartHardware BLUETOOTH_UART = {USART2, PA_2, PA_3,
                                            GPDMA1_Channel3, GPDMA1_REQUEST_USART2_TX,
                                            GPDMA1_Channel2, GPDMA1_REQUEST_USART2_RX};
constexpr uint32_t BLUETOOTH_BAUD = 115200;   // TODO: match the module's configured rate

} // namespace board

#endif // BUCKY_BOARD_H562RG_H
