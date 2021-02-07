# ArduinoSnakeRobot
This was a project to develop a Concentric tube robot at IIT Madras using an Arduino Mega to control several stepper motors and limit switches for inverse kinematic control of the snake robot.

![alt text](https://github.com/Andrew-Raz-ACRV/ArduinoSnakeRobot/blob/main/20180608_103504.jpg)

## Getting Started with the Project
You'll need these libraries for the Arduino:
AccelStepper for Stepper motor control:

```
#include <AccelStepper.h> //Get from https://github.com/adafruit/AccelStepper
```

And MatrixMath for the inverse kinematics of a concentric tube robot

```
#include <MatrixMath.h> //Get from https://github.com/eecharlie/MatrixMath
```
## Final Code
There are a bunch of MATLAB code to interract with the robot including a GUI that visualises the robot and serial communication for inverse kinematics commands
Run the main arduino code:

```
SnakeRobotIITMadrasSerialInterface.ino
```

And on Matlab run the GUI:

```
SnakeRobotIITMadrasGUI.m
```

## Arduino Setup:
You'll need power for the stepper motors and limit switches for the concentric tube self-collisions
Interrupts have been used to stop motor control in this way.

The Pin connections: (sorry its just a picture rather than a diagram)
![alt text](https://github.com/Andrew-Raz-ACRV/ArduinoSnakeRobot/blob/main/20180531_144000.jpg)
