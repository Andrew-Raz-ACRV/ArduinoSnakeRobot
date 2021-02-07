// Blocking.pde
// -*- mode: C++ -*-
//
// Shows how to use the blocking call runToNewPosition
// Which sets a new target position and then waits until the stepper has 
// achieved it.
//
// Copyright (C) 2009 Mike McCauley
// $Id: Blocking.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 2, 3); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
//AccelStepper::setEnablePin(4);
//stepper.setOutputPins(4);

void setup()
{  
    Serial.begin(9600);
    pinMode(4,OUTPUT);
    stepper.setMaxSpeed(200.0);
    stepper.setAcceleration(100.0);
}

void loop()
{    
    
    Serial.println("Running to Position 0");
    stepper.runToNewPosition(0);
    Serial.println("Running to Position 500");
    stepper.runToNewPosition(500);
    Serial.println("Running to Position 100");
    stepper.runToNewPosition(100);
    Serial.println("Running to Position 120");
    stepper.runToNewPosition(120);
    //stepper.disableOutputs();
    digitalWrite(4,LOW);
}
