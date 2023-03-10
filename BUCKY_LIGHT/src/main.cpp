#include <Arduino.h>
#include <TSSP2.h>

int pins[16] = {14, 27, 26, 25, 33, 32, 35, 34, 23, 22, 21, 19, 18, 17, 16, 15};
TSSP2 tssp2(pins);

void setup() {
  tssp2.setAllSensorPinsInput();
   Serial.begin(115200);
   pinMode(13, OUTPUT);
   digitalWrite(13, HIGH);
}

void loop() {
  tssp2.getAllSensorPulseWidth(833);
  tssp2.calcVector();
  tssp2.calcRTfromXY();
  Serial.println(tssp2.IRInfo_theta);   
  delay(100);  
}