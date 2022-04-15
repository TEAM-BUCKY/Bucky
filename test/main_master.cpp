// Master

#include <Arduino.h>
#include <Motor.h>
#include <Core.h>
#include <Wire.h>
#include <TSSP2.h>
#include <Serial_C.h>
#include <Compass.h>
#include <TeensyThreads.h>

// pins list
int pins[14] = {17, 16, 14, 40, 38, 39, 41, 15, 18, 24, 26, 27, 25, 19};

const float heading_x_const = 1.00;

float calcPIDReturn;

// make class objects
Compass magn(1000);
TSSP2 ir(pins);
Serial_C serial (true);



void CalcPID(int dummyArg)
{ // thread function
  float x_ir = ir.IRInfo_x;
  calcPIDReturn =  x_ir * heading_x_const;
}

void setup() // setup function
{
  Serial.begin(115200); // begin Serial
  serial.setup(); // setup serial for communication with slave
  ir.setAllSensorPinsInput(); // set all the sensor pins to input
}

void loop()
{
    threads.addThread(CalcPID);

    // ir functions
    ir.getAllSensorPulseWidth(833);
    ir.calcRTfromXY();
    ir.calcVector();
  
}