#include <Arduino.h>
#include <Serial_C.h>
#include <stdlib.h>

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
    Serial.begin(115200);
}

void Serial_C::send(float f1, float f2) {
    if (isMaster) {
        Serial5.print(String(f1) + ',' + String(f2) + '\r');
    }

}

void Serial_C::receive() {
    if (isMaster) {
        receive_string = Serial5.readStringUntil('\r');
        int string_length = receive_string.length();
        for (int i = 0; i< string_length; i++) {
            if (receive_string[i] == ',') {
                theta = receive_string.substring(0, i).toFloat();
                x = receive_string.substring(i+1, string_length-i).toFloat();
                break;
            }
        }
    }
    else {
        receive_string = Serial3.readStringUntil('\r');
        int string_length = receive_string.length();
        for (int i = 0; i< string_length; i++) {
            if (receive_string[i] == ',') {
                theta = receive_string.substring(0, i).toFloat();
                x = receive_string.substring(i+1, string_length-i).toFloat();
                break;
            }
        }
    }
}
