// Master

#include <Arduino.h>
#include <Motor.h>
#include <Core.h>
#include <Wire.h>
#include <TSSP2.h>
#include <Serial_C.h>
#include <TeensyThreads.h>

// pins list
int pins[14] = {17, 16, 14, 40, 38, 39, 41, 15, 18, 24, 26, 27, 25, 19};

const float heading_x_const = 1.00;
String defaultString = "000.00000.00000.00+++\r";

float calcPIDReturn;

// make class objects
TSSP2 ir(pins);
Serial_C serial(true);

void CalcPID(int dummyArg)
{ // thread function
  float x_ir = ir.IRInfo_x;
  calcPIDReturn = x_ir * heading_x_const;

}

void calcSpeed() {
  defaultString = "000.00000.00000.00+++\r";
  double degrees = ir.IRInfo_theta;
  double speedM1_ = -(100)*sin((degrees + 180) / PI);
  double speedM2_ = -(100)*sin((degrees + 60) / PI);
  double speedM3_ = -(100)*sin((degrees - 60) / PI);

  double speeds_[3] = {speedM1_, speedM2_, speedM3_};

  for (int i = 0; i < 3; i++) {
    if (speeds_[i] < 0) {
      defaultString[18+i] = '-';
    }
    if (speeds_[i] >= 0) {
      defaultString[18+i] = '+';
    }

  }

  String speedM1 = String(abs(speedM1_), 2);
  String speedM2 = String(abs(speedM2_), 2);
  String speedM3 = String(abs(speedM3_), 2);


  String speeds[3] = {speedM1, speedM2, speedM3};
  for (int i = 0; i < 3; i++) {
    String str_ = speeds[i];
    for (int x = 0; x < str_.length(); x++) {
      defaultString[((i+1)*6)-x-1] = str_[str_.length()-1-x];
    }
  } 

}

void setup() // setup function
{
  Serial.begin(115200);       // begin Serial
  serial.setup();             // setup serial for communication with slave
  ir.setAllSensorPinsInput(); // set all the sensor pins to input
}

void loop()
{
  //threads.addThread(CalcPID);
  calcSpeed();
  
  // ir functions
  ir.getAllSensorPulseWidth(833);
  ir.calcRTfromXY();
  ir.calcVector();
  Serial.println(ir.IRInfo_theta);

}