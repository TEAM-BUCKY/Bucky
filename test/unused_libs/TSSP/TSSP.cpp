#include <Arduino.h>
#include <TSSP.h>

void setAllSensorPinsInput(void){
  for(int i = 0; i < IR_NUM; i++){
    pinMode(SensorPins[i], INPUT);
  }
}

bool getSensorPin(uint8_t pin){
  switch(pin){
    case 0:   return digitalRead(17);
    case 1:   return digitalRead(16);
    case 2:   return digitalRead(14);
    case 3:   return digitalRead(40);
    case 4:   return digitalRead(38);
    case 5:   return digitalRead(39);
    case 6:   return digitalRead(41);
    case 7:   return digitalRead(15);
    case 8:   return digitalRead(18);
    case 9:   return digitalRead(24);
    case 10:   return digitalRead(26);
    case 11:   return digitalRead(27);
    case 12:   return digitalRead(25);
    case 13:   return digitalRead(19);
  }
}

sensorInfo_t getAllSensorPulseWidth ( float pulseWidth [IR_NUM], uint16_t timeLimit) {
    // Structure to store experimental data
    sensorInfo_t sensorInfo;

    // pulseWidth [] is a variable for addition calculation, so initialize it first.
    for(int i = 0; i < IR_NUM; i++) {
        pulseWidth[i] = 0;
    }

    // Read the sensor while monitoring the time (833us) with do-while
    const unsigned long startTime_us = micros();
    do {
        for (int i = 0; i < IR_NUM; i++) {
            if(getSensorPin(i) == false) {
                pulseWidth [i] += deltaPulseWidth;
            }
        
        }
    } while((micros() - startTime_us) < timeLimit);

    // If sensing is performed only by vector calculation, the following implementation is unnecessary.
    sensorInfo. maxPulseWidth      = 0 ; // Pulse width of the most responsive sensor
    sensorInfo. maxSensorNumber   = 0 ; // The most responsive sensor number
    for(int i = 0; i < IR_NUM; i++) {
        if(pulseWidth[i] > 0) {
            sensorInfo.activeSensors += 1;
        }
        if(pulseWidth[i] > sensorInfo.maxPulseWidth) {
            sensorInfo. maxPulseWidth = pulseWidth [i];
            sensorInfo.maxSensorNumber = i;
        }
    }

    return sensorInfo;
}