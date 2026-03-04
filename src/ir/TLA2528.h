#ifndef BUCKY_TLA2528_H
#define BUCKY_TLA2528_H

#include <Wire.h>

// TLA2528 I2C opcodes
#define TLA2528_OP_RREG  0x10
#define TLA2528_OP_WREG  0x08

// TLA2528 register addresses
#define TLA2528_SYSTEM_STATUS    0x00
#define TLA2528_GENERAL_CFG      0x01
#define TLA2528_DATA_CFG         0x02
#define TLA2528_OSR_CFG          0x03
#define TLA2528_OPMODE_CFG       0x04
#define TLA2528_PIN_CFG          0x05
#define TLA2528_GPIO_CFG         0x07
#define TLA2528_GPO_DRIVE_CFG    0x09
#define TLA2528_GPO_VALUE        0x0B
#define TLA2528_GPI_VALUE        0x0D
#define TLA2528_SEQUENCE_CFG     0x10
#define TLA2528_MANUAL_CH_SEL    0x11
#define TLA2528_AUTO_SEQ_CH_SEL  0x12

#define TLA2528_NUM_CHANNELS     8

class TLA2528 {
private:
    TwoWire* wire = nullptr;
    uint8_t addr;

    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);

public:
    explicit TLA2528(uint8_t i2cAddr);

    bool init(TwoWire& wireRef);
    uint16_t readChannel(uint8_t channel);
    void readAllChannels(uint16_t values[TLA2528_NUM_CHANNELS]);
};

#endif //BUCKY_TLA2528_H
