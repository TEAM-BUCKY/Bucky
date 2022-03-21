#include <Arduino.h>
#include <Serial_C.h>

Serial_C::Serial_C(bool isMaster){
    Serial_C::isMaster = isMaster;
}

void Serial_C::setup() {
    if (isMaster) {
        Serial5.begin(115200);
    }
    else {
        Serial3.begin(115200);
    }
}

void Serial_C::send(String message) {
    if (isMaster) {
        Serial5.print(message);
    }
    else {
        Serial3.print(message);
    }
}

String Serial_C::receive() {
    if (isMaster) {
        return Serial5.readStringUntil('\r');
    }
    else {
        return Serial3.readStringUntil('\r');
    }
}
