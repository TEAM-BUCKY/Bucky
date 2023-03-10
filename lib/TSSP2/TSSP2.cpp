#include <Arduino.h>
#include <TSSP2.h>


TSSP2::TSSP2(int pins[16]) { // equal the argument array to array in class
    for (int i =0; i < TSSP2::ir_num; i++) {
        TSSP2::pins[i] = pins[i];
    }
}

void TSSP2::setAllSensorPinsInput() { // set all the sensorpins to input
    for (int i = 0; i < ir_num; i++) {
        pinMode(pins[i], INPUT);
    }
}

bool TSSP2::getSensorPin(int pin) { // get digitalread of pins specified
    return digitalRead(pins[pin]);
}

void TSSP2::getAllSensorPulseWidth(int timeLimit) { // get the pulsewidth of every sensor
    for (int i = 0; i < ir_num; i++) { // set the pulsewidth array to 0
        pulseWidth[i] = 0; 
    }

    const long startTime_us = micros(); // get the starttime
    do { // while inside of the timelimit
        for (int i = 0; i < ir_num; i++) {
            if (getSensorPin(i) == false) { // if a sensor reads false (its inverted so when it detects something its false)
                pulseWidth[i] += deltaPulseWidth; // add pulsewidth every cycle
            }
        }
    } while ((micros() - startTime_us) < timeLimit);

    maxPulseWidth = 0; // reset max pulse width

    maxSensorNumber = 0; // reset max sensor number

    activeSensors = 0; // reset active sensors
    for (int i = 0; i < ir_num; i++) {
        if (pulseWidth[i] > 0) {
            activeSensors += 1; // get amount of active sensors
        }
        if (pulseWidth[i] > maxPulseWidth) { // get highest pulsewidth reading and get the sensor with the highest reading
            maxPulseWidth = pulseWidth[i];
            maxSensorNumber = i;
        }
    }
}

void TSSP2::calcVector() { // calculate the vector with the pulsewidth of every sensor
    IRInfo_x = 0.0;
    IRInfo_y = 0.0;
    for (int i = 0; i < ir_num; i++) {
        IRInfo_x += pulseWidth[i] * unitVectorX[i]; 
        IRInfo_y += pulseWidth[i] * unitVectorY[i];
    }
}


void TSSP2::calcRTfromXY() { // calculate the radius and theta with vector of x, y
    IRInfo_radius = sqrt(pow(IRInfo_x, 2.0) + pow(IRInfo_y, 2.0));
    IRInfo_theta = atan2(IRInfo_x, IRInfo_y) / PI * 180.0;
}
