# Team Bucky, Robocup Junior
The better iteration of Goaly Robot

# Table of Contents 
- Introduction
- Hardware
  - Pcb
- Software
  - Conventions

- History

# Hardware
- ESP32
  - Is basically the mini computer, has everything you need. And it also has wifi and bluetooth. 
- Shield
  - The shield is basically the extension of the core: A board with a specific form-factor that allows it to plug directly into an ESP32 board to add new functionality, such as Wi-Fi, Ethernet, motor control, or sensors, without complex wiring. 
  - On both sides of the shield you can plug in motor control, IR-sensors and line sensors and we even have some analogue and digital pins.
  - So the shield receives every input, sends it all to the ESP32 to process, and then the ESP32 sends a signal back to the shield to control the motors and more.
- Line sensor
  - On both sides are connections so you can make a chain of line sensors, you have one master line sensor, in there you plug in all the others, but only on one side of the master line sensor you can plug in the chain. it is still recommended to only chain 8 sensors, not more. They detect a line by sending a pulse red light signal using a photo transistor we can read the signal we get back. If we see a black line no light will reflect back, and if we see a white line a lot will reflect. You can adjust the threshold to adjust when it detects a line for how much light by adjusting a variable resistor. This gives a digital signal, you can use this as interrupts so you can react to a line even without checking specifically in your code.
- Motor driver
  - You have 6 pwm pins to control three motors in both forward and reversed direction.
- The IR sensor + Controller
  - The IR controller provides 16 ports for IR sensors, these first take in a IR signal, pulsed at 40 kHz, and when the signal reaches the controller, they sample that and then we hold the highest value untill we receive the next set of pulses. So you can always read even if there are no pulses. The analogue signal is fed into a analogue to digital converter so you can read the analogue signal. we need this because the ball sends out 8 pulses at 40 kHz, then it waits a bit and then repeat. so we need to hold the signal so we can read it out. <img width="2862" height="734" alt="image" src="https://github.com/user-attachments/assets/9252f6f5-7dc3-4fc0-a42b-5eafbf3a5f01" />

  - On IR-sensor it automatically filters so it only detects IR signal with a pusle of 40 kHz. 

# Software

## Conventions
All code uses the same frame, angle and ordering conventions. Code that doesn't follow them is a bug.

- **Axes**: `y` = forward, `x` = right. This holds for both the robot frame and the field frame (field `y` points toward the opponent goal).
- **Angles**: clockwise positive, `0` = forward (`+y`).
  - Direction of angle `a`: `(x, y) = (sin a, cos a)`
  - Angle of a vector: `a = atan2(x, y)` (**not** `atan2(y, x)`)
- **Rotation**: positive rotation = clockwise.
- **Ordering**: everything is numbered clockwise, seen from above: motors, IR sensors, sonars, line sensors.
  - IR sensor 0 and sonar 0 face straight forward (`0°`); the following sensors go clockwise (IR sensor `i` is at `i * 360° / count`, sonars are front, right, back, left).
- **Units**: distances in cm. The EKF/strategy code uses radians; compass/drive helpers use degrees (convert with `Math::degreesToRadians`).

### Motors
Seen from the robot's perspective, numbered clockwise:

| Motor | Position | Angle |
|-------|----------|-------|
| M1    | right    | 60°   |
| M2    | back     | 180°  |
| M3    | left     | 300°  |

A positive wheel speed in `Motors::wheelSpeeds` means the wheel pushes the robot clockwise.

### Compass / accelerometer (LSM303AGR)
Mount the chip facing up, with `x` to the right and `y` to the front. Its axes then match the robot frame, and the compass heading increases clockwise.
