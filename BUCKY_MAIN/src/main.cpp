#include <Arduino.h>
#include <CANWrapper.h>

CANWrapper CanIR;
CANWrapper CanLight;


void setup()
{
  CanIR.begin(500E3);
  CanIR.setPins(18, 16);
  Serial.begin(115200);
  //CanIR.setFilter(0);
}

void loop()
{
  char* data = CanIR.readData();
  Serial.println(data);
  delay(500);
}