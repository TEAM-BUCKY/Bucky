#include <Arduino.h>
#include <Motor.h>
#include <Wire.h>

Motor m1 (2, 3);
Motor m2 (4, 5);
Motor m3 (7, 6);

void move(int deg, int basespeed, float offset)
{ // move functions for motors (degrees from -180 to 180)
  float pi = 57.29577951;

  float speedM1 = -(basespeed)*sin((deg + 180) / pi);
  float speedM2 = -(basespeed)*sin((deg + 60) / pi);
  float speedM3 = -(basespeed)*sin((deg - 60) / pi);

  m1.move(speedM1, offset);
  m2.move(speedM2, offset);
  m3.move(speedM3, offset);
}

void setup() {
    m1.setup();
    m2.setup();
    m3.setup();

}

void loop() {
    move(0, 255, 0);
    delay(1);
    move(180, 255, 0);
    delay(1);
}