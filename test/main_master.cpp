// Master

#include <Arduino.h>
#include <Motor.h>
#include <Core.h>
#include <Wire.h>
#include <TSSP2.h>
#include <Serial_C.h>
#include <TeensyThreads.h>

int IR_pins[14] = {17, 16, 14, 40, 38, 39, 41, 15, 18, 24, 26, 27, 25, 19};


Serial_C serial (true);
TSSP2 tssp(IR_pins);


void setup() // setup function
{
  Serial.begin(115200);       // begin Serial
  serial.setup();             // setup serial for communication with slave
  tssp.setAllSensorPinsInput(); // set all the sensor pins to input
}

void loop()
{
  
  tssp.getAllSensorPulseWidth(833);
  tssp.calcRTfromXY();
  tssp.calcVector();
  serial.send(tssp.IRInfo_theta, tssp.IRInfo_x);

}