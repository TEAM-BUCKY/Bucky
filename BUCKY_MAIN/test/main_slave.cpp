// Slave

#include <Arduino.h>
#include <Wire.h>
#include <Core.h>
#include <HMC5883L_lib.h>
#include <LightGate.h>
#include <LightRing.h>
#include <Motor.h>
#include <Serial_C.h>
#include <Ultrasonic.h>
#include <LightGate.h>

LightGate lGate(21);

Motor m1(1, 2);
Motor m2(22, 23);
Motor m3(24, 25);

Serial_C serial(false);

HMC5883L_Calibrate calibration;
HMC5883L_Compass magnetomer(0.3, 1000);

Ultrasonic left(8, 9);
Ultrasonic right(39, 38);

bool outBounds = false;

int S[5] = {33, 34, 35, 36, 29}; // A0, A1, A2, A3, A4 Respectively on Arduino

int CS = 30;
int WR = 31;
int EN = 32;

int leftSum;
int rightSum;

int buttonPressedTimes = 0;

int speed = 0;

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

int invertDegrees(int deg)
{
  if (deg == 180 || deg == 0)
    deg += 180;
  while (1)
  {
    if (deg > 360)
      deg -= 360;
    if (deg < 0)
      deg += 360;

    if (deg > 0 && deg < 360)
      break;

    // Serial.println(deg);
  }

  return deg;
}

void move(int deg, int basespeed, float offset)
{ // move functions for motors (degrees from -180 to 180)
  float pi = 57.29577951;

  float speedM1 = -(basespeed)*sin((deg + 180) / pi);
  float speedM2 = -(basespeed)*sin((deg + 60) / pi);
  float speedM3 = -(basespeed)*sin((deg - 60) / pi);

  m1.move(speedM1, offset);
  m2.move(speedM2, offset);
  m3.move(speedM3, offset);
}

void setup()
{

  // button setup
  pinMode(28, INPUT);
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
  // Serial begins/setups
  Serial.begin(115200);
  serial.setup();
  // Motor setups
  m1.setup();
  m2.setup();
  m3.setup();
  // Ultrosonic setup
  calibration.setup();
  left.ultraSetup();
  right.ultraSetup();
  const long startTime = millis();
  do
  {
    calibration.calibrate();
  } while ((millis() - startTime) < 15000);
  magnetomer.setup(calibration.offX, calibration.offY);
  Serial.println("help");
  magnetomer.calibrate();
  for (int i = 0; i < 10; i++)
  {
    leftSum += left.read();
    rightSum += right.read();
  }
  leftSum = leftSum / 10;
  rightSum = rightSum / 10;

  // Lightgate setup
  lGate.setup();
}

void loop()
{
  // lightring checks
  int x = 0;
  int degrees = -1;
  for (int i = 0; i < 24; i += 2)
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
  if (degrees != -1)
  {
    move(degrees, 100, 0);
  }
  else
  {
    serial.receive();
    outBounds = false;
    // everything to do other then out of bounds
    Serial.println(left.read());
    // check if in lightgate
    // if ball in lightgate
      move(serial.theta * 1.5, 150, magnetomer.calculate()); // move towards ball
  }
}