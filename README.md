# RBCJr2022
The second iteration of Goaly Robot

# Table of Contents 
- Introduction
- Hardware
  - Pcb
- Software
  - [Compass](#Compass)
  - [Lightgate](#Lightgate)
  - [LightRing](#Lightring)
  - [Motor](#Motor)
  - [Serial](#Serial)
  - [TSSP](#TSSP)
  - [Ultrasonic](#Ultrasonic)
- History

# Hardware

# Software

## Compass


[Back to contents](#Table-of-Contents)
## Lightgate

[Back to contents](#Table-of-Contents)
## LightRing

[Back to contents](#Table-of-Contents)
## Motor

[Back to contents](#Table-of-Contents)
## Serial

[Back to contents](#Table-of-Contents)
## TSSP
We use TSSP sensors to locate the ball. The ball sends out infrared, and we can detect that with tssp sensors. 
These are the files:
[cpp file](lib/TSSP2/TSSP2.cpp)
[header](lib/TSSP2/TSSP2.h)

This is the init function. Here we make all the specified pin numbers global in the class.
```python
TSSP2::TSSP2(int pins[14]) { // equal the argument array to array in class
    for (int i =0; i < TSSP2::ir_num; i++) {
        TSSP2::pins[i] = pins[i];
    }
```
As you can see in the header, the pins are private in the tssp class.
`int pins[14];`

The next function will be used to set all the pins to the input mode. That is needed because we want to read the pins.
```python
void TSSP2::setAllSensorPinsInput() { // set all the sensorpins to input
    for (int i = 0; i < ir_num; i++) {
        pinMode(pins[i], INPUT);
    }
}
```
The ir_num is set in the header as private.
`int ir_num = 14;`

After that we have a function; `bool TSSP2::getSensorPin(int pin)`
The purpose of that function is to see if the specified pin returns false or true, while staying Object Orientated. This function will be used in function after this. It's also set to public in the header.

The next function is a bit complicated, but I'll try to talk you through it.
`void TSSP2::getAllSensorPulseWidth(int timeLimit)` is the function. `timeLimit` is an argument, because that may vary between different sensors. For us we use 833 microS. The first thing we run into, is a for loop:
```python
for (int i = 0; i < ir_num; i++) { // set the pulsewidth array to 0
        pulseWidth[i] = 0; 
    }
```
In here we set the pulseWidth for every pin to 0. After that, we'll get a starting time, which is needed in the future. Then, we have a while loop. It loops until the timeLimit has passed. In the while loop we have a for loop. 
```python
for (int i = 0; i < ir_num; i++) {
            if (getSensorPin(i) == false) { // if a sensor reads false (its inverted so when it detects something its false)
                pulseWidth[i] += deltaPulseWidth; // add pulsewidth every cycle
            }
        }
```
For every pin, we check if the pin is detecting something. If it detects something, we will add the deltaPulseWidth, which is set to 2. So the sensor where the ball is the closest to will have the biggest pulseWidth. So after this loop, we will be checking which sensor has the highest pulse width, and assigning it to a public variable, that way we can access it in the main file. 
After we've done this, we can just send the robot to the degree the sensor with the highest pulse width is, but that is not realy accurate. So we have two more functions to calculate almost the exact position of the ball, in theory it should be the exact.
So first we need to calculate the vectors to. We save those to `IRInfo_x` and `IRInfo_y`. We just multiply the pulseWidth and unitVectorX for every sensor. 
`UnitVectorX` is the Sin of the degrees of the sensor. `UnitVectorY` is the tan.
After that is done, we can use that info to calculate the radius and the theta. 
The radius is calculated like this; `sqrt(pow(IRInfo_x, 2.0) + pow(IRInfo_y, 2.0));`
The theta is calculated like thos; `atan2(IRInfo_y, IRInfo_x) / PI * 180.0;`
[Back to contents](#Table-of-Contents)
## Ultrasonic

[Back to contents](#Table-of-Contents)