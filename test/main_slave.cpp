// Slave

#include <Arduino.h>
#include <Motor.h>
#include <Core.h>
#include <Wire.h>
#include <Serial_C.h>
#include <HMC5883L_lib.h>
#include <Adafruit_NeoPixel.h>

#define LEDPIN 0
#define NUMLEDS 8

int buttonPressed = 0;
int pinButton = 28;
bool calibration = true;

Motor m1(1, 2);
Motor m2(22, 23);
Motor m3(24, 25);

Adafruit_NeoPixel pixels(NUMLEDS, LEDPIN, NEO_GRB + NEO_KHZ800);
Serial_C serial(false);
HMC5883L_Calibrate magCalibrate;
HMC5883L_Compass magCompass(0.037, 1000);

void interrupt_function()
{
  buttonPressed++;
}

void setup()
{
  pinMode(pinButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinButton), interrupt_function, RISING);
  Serial.begin(115200);
  Serial.println("start");
  serial.setup();
  m1.setup();
  m2.setup();
  m3.setup();

  pixels.begin();

  magCalibrate.setup();

  magCompass.setup(magCalibrate.offX, magCalibrate.offY);

  // wait for button press

  // calibrate towards goal

  // wait for button press -> loop
  pixels.clear();
}

void loop()
{
  if (calibration)
  {
    if (buttonPressed == 0)
    {
      pixels.setPixelColor(buttonPressed, pixels.Color(0, 150, 0));
      pixels.show();
    }
    else if (buttonPressed == 1)
    {
      pixels.setPixelColor(buttonPressed, pixels.Color(0, 150, 0));
      pixels.show();
    }
    else if (buttonPressed == 2)
    {
      pixels.setPixelColor(buttonPressed, pixels.Color(0, 150, 0));
      pixels.show();
    }
    else if (buttonPressed == 3)
    {
      pixels.setPixelColor(buttonPressed, pixels.Color(0, 150, 0));
      pixels.show();
    }
    else if (buttonPressed == 4)
    {
      pixels.setPixelColor(buttonPressed, pixels.Color(0, 150, 0));
      pixels.show();
    }
    else if (buttonPressed == 5)
    {
      pixels.setPixelColor(buttonPressed, pixels.Color(0, 150, 0));
      pixels.show();
    }
    else if (buttonPressed == 6)
    {
      pixels.setPixelColor(buttonPressed, pixels.Color(0, 150, 0));
      pixels.show();
    }
    else if (buttonPressed == 7)
    {
      pixels.setPixelColor(buttonPressed, pixels.Color(0, 150, 0));
      pixels.show();
    }
  }

  String receivedString = serial.receive(); // receive full string (in format: 000.00000.00000.00+++)

  float motorSpeeds[3] = {};

  for (int i = 1; i < 4; i++) {
    motorSpeeds[i-1] = receivedString.substring(i*6-6, i*6).toFloat(); 
    if (receivedString[receivedString.length()-3+(i-1)] < 0) {
      motorSpeeds[i-1]*-1;
    }
  }

  float offset = magCompass.calculate()*1.00; // change this
  m1.move(motorSpeeds[0], offset);
  m2.move(motorSpeeds[1], offset);
  m3.move(motorSpeeds[2], offset);
  
}
