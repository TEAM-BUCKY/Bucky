#include <Arduino.h>
#include <LightRing.h>

LightRing::LightRing(int pins[9])
{ // assign all the pins, see header for order
    for (int i = 0; i < 9; i++)
    {
        LightRing::pins[i] = pins[i];
    }
}

void LightRing::setup()
{
    for (int x = 0; x < 5; x++) {
    pinMode(pins[x], OUTPUT);
    digitalWrite(pins[x], LOW);
  }
  pinMode(pins[5], OUTPUT);
  digitalWrite(pins[5], LOW);
  pinMode(pins[6], OUTPUT);
  digitalWrite(pins[6], LOW);
  pinMode(pins[7], OUTPUT);
  digitalWrite(pins[7], LOW);
  pinMode(pins[8], INPUT);
}
void LightRing::pinSelect(int pin)
{ // select pin
    digitalWrite(31, LOW);
    for (int x = 0; x<5; x++){
      byte state = bitRead(pin, x);
      digitalWrite(pins[x], state);

    }
   
  digitalWrite(31, HIGH);
}

int LightRing::ringRead(int pin)
{   // read pin val
    // for (int i = 0; i < 28; i++) { // for loop, but skip 13-16
    //     if (i >= 13 && i<= 16) {
    //         continue;
    //     }
    //     pinSelect(i); // select pin i
    //     int val = analogRead(pins[8]); // assign val_ with analogRead
    //     Serial.println(String(i+1) + ": " + String(val));
    //     // if (val > 30) { // CHANGE VALUE WITH TESTING !!!!
    //         //return i * 15;
    //     //}
    // }
    pinSelect(pin);
    
    return analogRead(pins[8]);
}
