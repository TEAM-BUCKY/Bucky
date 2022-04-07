// CompassTest

#include <Arduino.h>
#include <Core.h>
#include <Wire.h>
#include <Compass.h>

Compass compass (0.038920842);

void setup() {
  Serial.begin(115200);
  compass.setup();
  compass.compassCalibrate(1000);
}

void loop() {
  Serial.println(compass.compassCalculate(compass.compassRead()));
}