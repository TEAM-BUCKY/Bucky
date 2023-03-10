#include <Arduino.h>
#include <CANWrapper.h>

CANWrapper Can;

void setup() {
  Can.begin(500E3);
  Serial.begin(115200);
}

void loop() {
  if (Can.available() > 0) {
    char* data = Can.readData();
    Serial.println(data);
  }
}