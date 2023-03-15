#include <Arduino.h>
#include <TSSP2.h>
#include <CANWrapper.h>

int pins[16] = {14, 27, 26, 25, 33, 32, 35, 34, 23, 22, 21, 19, 18, 17, 16, 15};
TSSP2 tssp2(pins);

void setup() {
    Serial.begin(115200);
    Serial.println(irTheta);
}

void loop(){

}