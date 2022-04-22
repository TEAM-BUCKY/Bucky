// Master

#include <Arduino.h>
#include <Motor.h>
#include <Core.h>
#include <Wire.h>
#include <TSSP2.h>
#include <Serial_C.h>
#include <TeensyThreads.h>

int IR_pins[14] = {17, 16, 14, 40, 38, 39, 41, 15, 18, 24, 26, 27, 25, 19};
float IR_calibration;

Serial_C serial (true);
TSSP2 tssp(IR_pins);


void setup() // setup function
{
  Serial.begin(115200);       // begin Serial
  serial.setup();             // setup serial for communication with slave
  tssp.setAllSensorPinsInput(); // set all the sensor pins to input
  
  for (int i = 0; i < 1000 ; i++) {

    tssp.getAllSensorPulseWidth(833);
    tssp.calcRTfromXY();
    tssp.calcVector();
    IR_calibration += tssp.IRInfo_theta;
  }
  IR_calibration = IR_calibration/1000;
}

void loop()
{
  tssp.getAllSensorPulseWidth(833);
  tssp.calcRTfromXY();
  tssp.calcVector();
  Serial.println(tssp.IRInfo_theta - IR_calibration);
  serial.send(tssp.IRInfo_theta - IR_calibration, tssp.IRInfo_x);

}