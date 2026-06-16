#include "tests.h"
#include "debug.h"
#include <Arduino.h>

static void scanBus(const char* name, const I2CDMABus& bus) {
    DBG_PRINT("--- ");
    DBG_PRINT(name);
    DBG_PRINTLN(" ---");

    uint8_t found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_dma_probe(&bus, addr)) {
            DBG_PRINT("  0x");
            if (addr < 0x10) DBG_PRINT('0');
            DBG_PRINT(addr, HEX);

            switch (addr) {
                case 0x10: DBG_PRINT("  (TLA2528 ADC1)"); break;
                case 0x14: DBG_PRINT("  (TLA2528 ADC2)"); break;
                case 0x19: DBG_PRINT("  (Unknown - Accel?)"); break;
                case 0x1E: DBG_PRINT("  (LIS2MDL Compass)"); break;
                default: break;
            }
            DBG_PRINTLN();
            found++;
        }
    }

    if (found == 0) {
        DBG_PRINTLN("  No devices found.");
    } else {
        DBG_PRINT("  ");
        DBG_PRINT(found);
        DBG_PRINTLN(" device(s) found.");
    }
    DBG_PRINTLN();
}

void testI2CScan(const TestContext& ctx) {
    DBG_PRINTLN();
    DBG_PRINTLN("=== I2C Bus Scanner ===");
    DBG_PRINTLN("Scanning all configured buses...");
    DBG_PRINTLN();

    scanBus("BUS1 (SDA=PB9, SCL=PA15)", ctx.i2c);

    DBG_PRINTLN("Scan complete. Rescanning every 5 seconds...");
    DBG_PRINTLN();

    while (true) {
        delay(5000);
        DBG_PRINTLN("--- Rescan ---");
        scanBus("BUS1", ctx.i2c);
    }
}
