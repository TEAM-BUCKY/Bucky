// include
#include <Arduino.h>
#include <Core.h>
#include <Ultrasonic.h>

// def var
long time; // time it takes in total for sound wave to travel back and forth
int distance; // distance of object relative to supersonic in cm

Ultrasonic::Ultrasonic(int trigger, int echo) {
    Ultrasonic::trigger = trigger;
    Ultrasonic::echo = echo;
}

void Ultrasonic::ultraSetup() {
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
    digitalWrite(trigger, LOW); 
}


int Ultrasonic::read() { // calculates and returns distance variab
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
    time = pulseIn(echo, HIGH);
    distance = time * 0.034 / 2;
    return distance;
}