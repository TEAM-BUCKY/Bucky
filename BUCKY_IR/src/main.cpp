#include <Arduino.h>
#include <TSSP2.h>
#include <CANWrapper.h>

int pins[16] = {14, 27, 26, 25, 33, 32, 35, 34, 23, 22, 21, 19, 18, 17, 16, 15};
TSSP2 tssp2(pins);
CANWrapper Canny; 

void setup() {
    tssp2.setup();
    Canny.setup();
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
}

void loop(){
    tssp2.update(833);
    Canny.sendDouble(tssp2.IRInfo_theta, 0);
    Canny.sendDouble(tssp2.IRInfo_radius, 1);
    delay(100);
}