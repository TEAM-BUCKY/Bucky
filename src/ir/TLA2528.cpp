#include "TLA2528.h"

TLA2528::TLA2528(uint8_t i2cAddr) : addr(i2cAddr) {}

void TLA2528::writeReg(uint8_t reg, uint8_t value) {
    wire->beginTransmission(addr);
    wire->write(TLA2528_OP_WREG);
    wire->write(reg);
    wire->write(value);
    wire->endTransmission();
}

uint8_t TLA2528::readReg(uint8_t reg) {
    wire->beginTransmission(addr);
    wire->write(TLA2528_OP_RREG);
    wire->write(reg);
    wire->endTransmission(false);
    wire->requestFrom(addr, (uint8_t)1);
    return wire->read();
}

bool TLA2528::init(TwoWire& wireRef) {
    wire = &wireRef;

    // Software reset
    writeReg(TLA2528_GENERAL_CFG, 0x01);
    delay(10);

    // Verify communication by reading back a known register
    uint8_t cfg = readReg(TLA2528_GENERAL_CFG);
    if (cfg == 0xFF) {
        return false; // No device responding
    }

    // All pins as analog inputs (default)
    writeReg(TLA2528_PIN_CFG, 0x00);

    // Manual channel selection mode
    writeReg(TLA2528_SEQUENCE_CFG, 0x00);

    return true;
}

uint16_t TLA2528::readChannel(uint8_t channel) {
    if (channel >= TLA2528_NUM_CHANNELS) return 0;

    // Select channel
    writeReg(TLA2528_MANUAL_CH_SEL, channel);

    // Read 2 bytes of conversion data
    wire->requestFrom(addr, (uint8_t)2);
    if (wire->available() < 2) return 0;

    uint8_t msb = wire->read();
    uint8_t lsb = wire->read();

    // 12-bit data: MSB[7:0] = DATA[11:4], LSB[7:4] = DATA[3:0]
    return ((uint16_t)msb << 4) | (lsb >> 4);
}

void TLA2528::readAllChannels(uint16_t values[TLA2528_NUM_CHANNELS]) {
    for (uint8_t ch = 0; ch < TLA2528_NUM_CHANNELS; ch++) {
        values[ch] = readChannel(ch);
    }
}
