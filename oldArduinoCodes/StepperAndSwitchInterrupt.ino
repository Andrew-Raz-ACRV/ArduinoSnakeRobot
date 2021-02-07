// Tube limit Test
// This causes a tube to move to its positive limit and then to its negative limit

#include <AccelStepper.h>

// Define a The Prismatic stepper motors
AccelStepper stepper1(AccelStepper::DRIVER, 22, 24);
AccelStepper stepper2(AccelStepper::DRIVER, 30, 32);

// Define the pin values for the switches, they are interrupt enabled according to this:
// https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
const byte switch2 = 18;
const byte switch3 = 3;

// Initially set the target to push the tube forwards for a long distance
double target = -1000000;

void setup() {
  Serial.begin(9600);

  //Switch interrupt initialisation
  pinMode(switch2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(switch2), collision_interrupt1, FALLING);
  pinMode(switch3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(switch3), collision_interrupt2, FALLING);

  //StepperMotor
  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(300.0);
  pinMode(26,OUTPUT);
  
  //StepperMotor
  stepper2.setMaxSpeed(1000.0);
  stepper2.setAcceleration(300.0);
  pinMode(34,OUTPUT);
    
}



void loop() {
  //start with delay before moving
  delay(5000); 

  //move second tube towards the target forever
  Serial.println("Working");
  stepper2.runToNewPosition(target); // blocking until tube reaches target
}



//interrupt for first switch:
// Going Forward

void collision_interrupt1()
{

  //Count time of the interrupt to deal with switch bouncing
 static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 
 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 200) 
 {
      // If the target is in the backward direction, then ignore it as you are going away from front limit
      if (stepper2.distanceToGo() > 0){
        
      }
      else{
        //if the target is in the forward direction it must be changed to stop going too far forward
        stepper2.stop();
        target = -target;        
      }
 }
  //Remember the time of the interrupt for debouncing
 last_interrupt_time = interrupt_time;
}

//interrupt for first switch:
// Going Backward


void collision_interrupt2()
{
    //Count time of the interrupt to deal with switch bouncing
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
 
 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 200) 
 {
      // If the target is in the forward direction, then ignore it as you are going away from back limit
      if (stepper2.distanceToGo() < 0 ){
        
      }
      else{
        //if the target is in the backward direction it must be changed to stop going too far back
        stepper2.stop();
        target = -target;        
      }

 }
  //Remember the time of the interrupt for debouncing
 last_interrupt_time = interrupt_time;
}



