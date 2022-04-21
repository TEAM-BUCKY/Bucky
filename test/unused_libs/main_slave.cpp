// Slave

#include <Arduino.h>
#include <Core.h>
#include <Wire.h>
#include <Serial_C.h>
#include <LightRing.h>
#include <Motor.h>

Motor m1 (1,2);
Motor m2 (22, 23);
Motor m3 (24, 25);

Serial_C serial (false);

bool outBounds = false;

int S[5] = {33, 34, 35, 36, 29}; // A0, A1, A2, A3, A4 Respectively on Arduino

int CS = 30;
int WR = 31;
int EN = 32;

int INPIN = 27; // IO PIN ON MEGAMUX

void pinSelect(int pinnum) // select pin for lightring
{
  digitalWrite(WR, LOW);
  for (int x = 0; x < 5; x++)
  {
    byte state = bitRead(pinnum, x);
    digitalWrite(S[x], state);
  }

  digitalWrite(WR, HIGH);
}


int invertDegrees(int deg) {
  if (deg == 180 || deg == 0) deg += 180;
  while (1) {
    if (deg > 360) deg -= 360;
    if (deg < 0) deg += 360;

    if (deg > 0 && deg < 360) break;

    // Serial.println(deg);

  }

  return deg;
}

void move(int deg, int basespeed) { // move functions for motors
  float pi = 57.29577951; 
  float speedM1 = -(basespeed) * sin((deg + 180) / pi);
  float speedM2 = -(basespeed) * sin((deg + 60) / pi); 
  float speedM3 = -(basespeed) * sin((deg - 60) / pi); 

  m1.move(speedM1, 0);
  m2.move(speedM2, 0);
  m3.move(speedM3, 0);

}


void setup()
{
  // setup for lightring
  for (int x = 0; x < 5; x++)
  {
    pinMode(S[x], OUTPUT);
    digitalWrite(S[x], LOW);
  }
  pinMode(CS, OUTPUT);
  digitalWrite(CS, LOW);
  pinMode(WR, OUTPUT);
  digitalWrite(WR, LOW);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);
  pinMode(INPIN, INPUT);

  // serial begins/setups
  Serial.begin(115200);
  serial.setup();


  // motor setups
  m1.setup();
  m2.setup();
  m3.setup();
}

void loop()
{


  // lightring checks 
  int x;
  int degrees = -1;
  for (int i = 0; i < 24; i++)
  {
    x = i;
    if (i > 12)
    {
      x = i + 4;
    }
    pinSelect(x);
    int val = analogRead(INPIN);
    if (val > 550)
    {
      // Serial.println("out of bounds");
      if (outBounds == false)
        degrees = invertDegrees(i * 15);
      outBounds = true;
      break;
    }
  }
  if (degrees != -1) {
    move(degrees, 100);
  } else {
    serial.receive();
    // everything to do other then out of bounds
    outBounds = false;
    move(serial.theta, 100);
  }
}