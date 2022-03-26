#include <Arduino.h>
#include <Motor.h>
#include <Core.h>
#include <Wire.h>
#include <Serial_C.h>
// #include <LSM9DS1.h>

Motor m1 (1, 2);
Motor m2 (22, 23);
Motor m3 (24, 25);

Lsm9ds1 gyro (300, -8.58);

Serial_C serial (false);

void move(float degrees, int basespeed) {
  float pi = 57.29577951; // 1: 240, 2: 120, 3: 0
  // float speedM1 = (cos((120-degrees) * pi)*basespeed);
  // Serial.print("motor 1: ");
  // Serial.println(speedM1);
  // float speedM2 = (cos((240-degrees) * pi)*basespeed);
  // Serial.print("motor 2: ");
  // Serial.println(speedM2);
  // float speedM3 = (cos((0-degrees) * pi)*basespeed);
  // Serial.print("motor 3: ");
  // Serial.println(speedM3);

  float speedM1 = -(basespeed) * sin((degrees + 180) / pi);
  float speedM2 = -(basespeed) * sin((degrees + 60) / pi);
  float speedM3 = -(basespeed) * sin((degrees - 60) / pi);

  m1.move(speedM1);
  m2.move(speedM2);
  m3.move(speedM3);
}

void setup() {
  Serial.begin(115200);
  Serial.println("start");
  serial.setup();
  // gyro.setup();
  // gyro.magCalibrate();
  m1.setup();
  m2.setup();
  m3.setup();
}

void loop() {
  float deg = serial.receive().toFloat();
  //Serial.println(gyro.magCalculate());
  move(deg, 0);
}
