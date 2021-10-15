#include <Arduino.h>
#include <Motor.h>
#include <Core.h>
#include <Wire.h>

Motor m1 (23, 25, 4);
Motor m2 (31, 33, 10);
Motor m3 (29, 27, 7);

enum Task {
  SLEEP,
  MOVE
};
Task task = MOVE;
bool reset = false;

void setup() {
  setupCore();
  analogReference(3.3);
  m1.setup();
  m2.setup();
  m3.setup();
  attachInterrupt(digitalPinToInterrupt(3), [](){reset = !reset;}, CHANGE);
}

void move(double degrees, int basespeed) {
  float pi = 57.29577951;
  float speedM1 = -(basespeed) * sin((degrees + 180) / pi);
  float speedM2 = -(basespeed) * sin((degrees + 60) / pi);
  float speedM3 = -(basespeed) * sin((degrees - 60) / pi);
  m1.move(speedM1);
  m2.move(speedM2);
  m3.move(speedM3);
}

void loop() {
  if(reset) {
    task = SLEEP;
  }
  switch(task) {
    case MOVE: {
      move(0, 255);
      delayMicroseconds(5);
      break;
    }
    case SLEEP: {
      m1.move(0);
      m2.move(0);
      m3.move(0);
      task = MOVE;
      delayMicroseconds(5);
      break;
    }
  }
}
