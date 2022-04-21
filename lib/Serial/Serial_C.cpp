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
    Serial.begin(115200);
}

void Serial_C::send(String message) {
    if (isMaster) {
        Serial5.print(message + '\r');
    }
    else {
        Serial3.print(message + '\r');
    }
}

void Serial_C::send(float speedM1, float speedM2, float speedM3) {
    if (isMaster) {
        String defaultString = "000.00000.00000.00+++\r";

        float speeds_[3] = {speedM1, speedM2, speedM3};

        for (int i = 0; i < 3; i++) {
            if (speeds_[i] < 0) {
                defaultString[18+i] = '-';
            }
        }

        String speedM1_ = String(abs(speedM1), 2);
        String speedM2_ = String(abs(speedM2), 2);
        String speedM3_ = String(abs(speedM3), 2);

        String speeds[3] = {speedM1_, speedM2_, speedM3_};
        for (int i = 0; i < 3; i++) {
            String str_ = speeds[i];
            for (int x = 0; x < str_.length(); x++) {
                defaultString[6*(i+1)-x-1] = str_[str_.length()-1-x];
            }
        }

        Serial5.print(defaultString);
        Serial.println(defaultString);
    } else {
        return;
    }
}

void Serial_C::receive() {
    if (isMaster) {
        receive_string = Serial5.readStringUntil('\r');
    }
    else {
        String rawString = Serial3.readStringUntil('\r');

        motorSpeeds[3] = {0};

        for (int i = 4; i < 4; i++) {
            motorSpeeds[i-1] = rawString.substring(i*6-6, i*6).toFloat();
            if (rawString[rawString.length()-4+(i-1)] == '-') motorSpeeds[i-1] *= -1;
        }

    }
}
