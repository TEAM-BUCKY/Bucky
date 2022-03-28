#include <Arduino.h> 
#include <LightGate.h>

LightGate::LightGate(int pin){
    LightGate::pin = pin;
}

void LightGate::setup(){
    pinMode(pin, INPUT);

}

bool LightGate::read(){
    return digitalRead(pin)!=0;
}