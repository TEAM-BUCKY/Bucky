
#include <Motor.h>
#include <Core.h>
#include <Wire.h>

Motor m1 (23, 25, 4);
Motor m2 (31, 33, 10);
Motor m3 (29, 27, 7);

void setup() {
  setupCore();
  analogReference(3.3);
}

void loop() {
}
