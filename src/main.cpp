#include <HardwareSerial.h>
#include <Arduino.h>


int main() {
    init();
    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }
    while (true) {
        Serial.println("Hello, World!!!");
        Serial.println("This is a test message.");
        delay(1000);
    }
    return 0;
}