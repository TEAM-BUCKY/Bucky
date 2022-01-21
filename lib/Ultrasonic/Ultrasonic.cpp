// include
#include <Arduino.h>
#include <Core.h>
#include <Ultrasonic.h>

// def var
long time; // time it takes in total for sound wave to travel back and forth
int distance; // distance of object relative to supersonic in cm

Ultrasonic::Ultrasonic(int in1, int in2) {
    Ultrasonic::in1 = in1;
    Ultrasonic::in2 = in2;
}

void Ultrasonic::setup() {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    digitalWrite(in1, LOW); 
}


int Ultrasonic::get() { // calculates and returns distance variab
    digitalWrite(in1, HIGH);
    delayMicroseconds(10);
    digitalWrite(in1, LOW);
    time = pulseIn(in2, HIGH);
    distance = time * 0.034 / 2
    return distance;
}