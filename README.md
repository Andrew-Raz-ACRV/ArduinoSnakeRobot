# ArduinoSnakeRobot

Embedded control for a three-tube concentric tube robot: an Arduino Mega driving seven stepper motors with limit-switch protection, and a MATLAB GUI that visualises the robot and sends inverse-kinematic commands over serial.

Built during a research visit to **IIT Madras in 2018**, modelled on the concentric tube robot at QUT.

**Status:** archived. Complete as of the 2018 visit and not maintained.

![The concentric tube robot built at IIT Madras](20180608_103504.jpg)

## How it works

A concentric tube robot is steered by rotating and translating pre-curved tubes relative to one another, so the control problem is coordinating every tube axis at once to hit a commanded tip pose.

**Three tubes, seven motors.** Each tube gets two — one to rotate it, one to translate it — for six axes of shape control, plus a seventh driving the gripper.

- **On the Arduino:** stepper coordination and the inverse kinematics, solved on-device using matrix arithmetic.
- **On the host:** a MATLAB GUI that renders the robot's configuration and sends target poses over the serial link.
- **Limit switches on interrupts.** Concentric tubes can collide with themselves when the axes run past one another. The switches are wired to hardware interrupts so motion halts immediately rather than at the end of the current control cycle — a software poll is not fast enough to protect the mechanism.

## Requirements

Two Arduino libraries, neither vendored here:

```cpp
#include <AccelStepper.h>   // https://github.com/waspinator/AccelStepper
#include <MatrixMath.h>     // https://github.com/eecharlie/MatrixMath
```

<!-- The original README pointed AccelStepper at github.com/adafruit/AccelStepper.
     Adafruit does host a fork, so the link was not broken — but AccelStepper is Mike
     McCauley's library (airspayce.com/mikem/arduino/AccelStepper/) and the widely
     used mirror is waspinator's. Either canonical source is better than a vendor
     fork for a library the reader will want the current version of.
     NEEDS ANDREW: if the Adafruit fork was used deliberately and the code depends on
     something specific to it, put that link back and say why. -->

Plus MATLAB on the host for the GUI.

## Running it

Flash the Arduino:

```
SnakeRobotIITMadrasSerialInterface.ino
```

Then start the GUI in MATLAB:

```matlab
SnakeRobotIITMadrasGUI.m
```

The steppers need their own power supply — the Arduino cannot drive them from board power.

## Wiring

![Pin connections for the stepper drivers and limit switches](20180531_144000.jpg)

A photograph of the actual wiring rather than a schematic. It is what exists.

## Related

The concentric tube robot at QUT that this one is modelled on, teleoperated by hand gesture: [hand_gesture_control_snakebot](https://github.com/Andrew-Raz-ACRV/hand_gesture_control_snakebot) — *Razjigaev et al., ROBIO 2017*.

## Licence

MIT — see [LICENSE](LICENSE).

## Questions

Written by Andrew Razjigaev. Questions: andrew_razjigaev@outlook.com
