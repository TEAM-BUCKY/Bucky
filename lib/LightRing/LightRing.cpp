#include <Arduino.h>
#include <LightRing.h>

LightRing::LightRing(int pins[9]){ // assign all the pins, see header for order
    for (int i = 0; i < 9; i++) {
        LightRing::pins[i] = pins[i];
    }
}

void LightRing::setup() { 
    for (int x = 0; x < 5; x++) { // set mode of select pins to output and write LOW
        pinMode(pins[x], OUTPUT);
        digitalWrite(pins[x], LOW);
  }
    pinMode(pins[5], OUTPUT); // CS to OUTPUT
    digitalWrite(pins[5], LOW);
    pinMode(pins[6], OUTPUT); // WR to OUTPUT
    digitalWrite(pins[6], LOW);
    pinMode(pins[7], OUTPUT); // EN to output
    digitalWrite(pins[7], LOW);
    pinMode(pins[8], INPUT); // INPUT-PIN to INPUT
}
void LightRing::pinSelect(int pin) { // select pin
    digitalWrite(pins[6], LOW);
    for (int x = 0; x<5; x++){
        byte state = bitRead(pin, x);
        digitalWrite(pins[x], state);

    }
   
    digitalWrite(pins[6], HIGH);

}


int LightRing::ringRead() { // read pin val
    int val = -1;
    int pin_;
    for (int i = 1; i < 29; i++) { // for loop, but skip 13-16
        if (i == 13) {
            i = i + 4;
        }
        LightRing::pinSelect(i); // select pin i
        int val_ = analogRead(pins[8]); // assign val_ with analogRead
        if (val_ > val && val_ > 999999999) { // CHANGE VALUE WITH TESTING !!!!
            val = val_;
            pin_ = i;
        }
    }
    int deg = pin_ * 15; // convert pin number to degrees
    

    // return deg when val is changed
    if (val != -1) {
        return deg;
    }
    return -1;
}