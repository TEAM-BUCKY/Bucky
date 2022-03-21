#include <Arduino.h>
#include <Motor.h>
#include <Core.h>
#include <Wire.h>
#include <TSSP.h>
#include <Serial_C.h>

unsigned long time_ms = 0;
#define T_MODEA 833

int serialPrintAllPusleWidth(float *pulseWidth, sensorInfo_t *infop) {
  return infop->maxSensorNumber;
}

Serial_C serial (true);

const int degree_increase = 25.7142857143;

void setup() {
  Serial.begin(115200);
  setAllSensorPinsInput();
  serial.setup();
}

void loop () {
  float           pulseWidth[IR_NUM]; // パルス幅を格納する変数
  sensorInfo_t    sensorInfo; 

  sensorInfo = getAllSensorPulseWidth(pulseWidth, T_MODEA);

  if (millis() - time_ms > 50) {
    time_ms = millis();
    
    int value = serialPrintAllPusleWidth(pulseWidth, &sensorInfo);
    if (value == 1 or value == 13) {
      value = 0;
    }
    serial.send(String(value * 25.7142857143));

    Serial.print(value);
    Serial.print("\n");
    }
}