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

```python
TSSP2::TSSP2(int pins[14]) { // equal the argument array to array in class
    for (int i =0; i < TSSP2::ir_num; i++) {
        TSSP2::pins[i] = pins[i];
    }
```
This is the init function. Here we make all the specified pin numbers global in the class.

`int pins[14];`
[Back to contents](#Table-of-Contents)
## Ultrasonic

[Back to contents](#Table-of-Contents)