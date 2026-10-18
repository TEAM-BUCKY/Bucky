#include <tests/tests.h>
#include <debug.h>
#include <Arduino.h>

static constexpr char MESSAGE[] = "Hello, World!\r\n";

void testBluetooth(const TestContext& ctx) {
    DBG_PRINTLN("=== Bluetooth Test ===");
    if (ctx.bluetooth == nullptr) {
        DBG_PRINTLN("This board has no Bluetooth module.");
        while (true) delay(1000);
    }
    HM10& bt = *ctx.bluetooth;
    DBG_PRINTLN(bt.configured() ? "HM-10 configured." : "HM-10 did not answer AT commands; assuming it is linked.");
    DBG_PRINTLN("Sending \"Hello, World!\" every 100 ms while a central is connected.");
    DBG_PRINTLN();

    uint32_t sent = 0, dropped = 0;
    uint32_t lastReport = millis();
    while (true) {
        if (bt.connected()) {
            if (bt.writeString(MESSAGE)) sent++;
            else dropped++;
        }

        // Echo anything the phone sends.
        while (bt.available()) DBG_PRINT(static_cast<char>(bt.read()));

        if (millis() - lastReport >= 1000) {
            lastReport += 1000;
            DBG_PRINT(bt.connected() ? "Connected" : "Waiting for connection");
            DBG_PRINT("\tSent: ");
            DBG_PRINT(sent);
            DBG_PRINT("\tDropped: ");
            DBG_PRINTLN(dropped);
        }

        delay(100);
    }
}
