#include "tests.h"
#include <Arduino.h>

static void scanBus(const char* name, TwoWire& wire) {
    Serial.print("--- ");
    Serial.print(name);
    Serial.println(" ---");

    uint8_t found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        wire.beginTransmission(addr);
        uint8_t err = wire.endTransmission();
        if (err == 0) {
            Serial.print("  0x");
            if (addr < 0x10) Serial.print('0');
            Serial.print(addr, HEX);

            // Label known devices
            switch (addr) {
                case 0x10: Serial.print("  (TLA2528 ADC1)"); break;
                case 0x14: Serial.print("  (TLA2528 ADC2)"); break;
                case 0x19: Serial.print("  (Unknown - Accel?)"); break;
                case 0x1E: Serial.print("  (LIS2MDL Compass)"); break;
            }
            Serial.println();
            found++;
        }
    }

    if (found == 0) {
        Serial.println("  No devices found.");
    } else {
        Serial.print("  ");
        Serial.print(found);
        Serial.println(" device(s) found.");
    }
    Serial.println();
}

void testI2CScan(MotorDriver&, Compass&, IRController&, I2CManager& i2c) {
    Serial.println();
    Serial.println("=== I2C Bus Scanner ===");
    Serial.println("Scanning all configured buses...");
    Serial.println();

    scanBus("BUS1 (SDA=PB9, SCL=PA15)", i2c.getBus(I2CBus::BUS1));
    scanBus("BUS3 (SDA=PC9, SCL=PC8)", i2c.getBus(I2CBus::BUS3));

    Serial.println("Scan complete. Rescanning every 5 seconds...");
    Serial.println();

    while (true) {
        delay(5000);
        Serial.println("--- Rescan ---");
        scanBus("BUS1", i2c.getBus(I2CBus::BUS1));
        scanBus("BUS3", i2c.getBus(I2CBus::BUS3));
    }
}
