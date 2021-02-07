// Tube limit Test
// This causes a tube to move to its positive limit and then to its negative limit

#include <AccelStepper.h>

// Define a The Prismatic stepper motors
AccelStepper stepper1(AccelStepper::DRIVER, 22, 24);
AccelStepper stepper2(AccelStepper::DRIVER, 30, 32);
AccelStepper stepper3(AccelStepper::DRIVER, 38, 40);

// Define the pin values for the switches, they are interrupt enabled according to this:
// https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
const byte switch1 = 2;
const byte switch2 = 18;
const byte switch3 = 3;
const byte switch4 = 19;

// Initially set the target to push the tube forwards for a long distance
double target1 = -1000000;
double target2 = 1000000;
double target3 = -1000000;

void setup() {
  Serial.begin(9600);

  //Switch interrupt initialisation
  pinMode(switch1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(switch1), collision_interrupt1, FALLING);
  pinMode(switch2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(switch2), collision_interrupt2, FALLING);
  pinMode(switch3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(switch3), collision_interrupt3, FALLING);
  pinMode(switch4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(switch4), collision_interrupt4, FALLING);
  
  //StepperMotor
  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(300.0);
  pinMode(26,OUTPUT);
  
  //StepperMotor
  stepper2.setMaxSpeed(1000.0);
  stepper2.setAcceleration(300.0); //300
  pinMode(34,OUTPUT);

    
  //StepperMotor
  stepper3.setMaxSpeed(1000.0);
  stepper3.setAcceleration(300.0);
  pinMode(42,OUTPUT);

}



void loop() {
  //start with delay before moving
  delay(5000); 

  while(1){
    //Kinematic control
    //Read motor values
    //q1 = prismatic_calibration*stepper1.currentPosition();
    //q2 = prismatic_calibration*stepper2.currentPosition();
    //q3 = prismatic_calibration*stepper3.currentPosition();

    //q4 = revolute_calibration*stepper4.currentPosition();
    //q5 = revolute_calibration*stepper4.currentPosition(); 
    //q6 = revolute_calibration*stepper4.currentPosition(); 

    //Compute Forward Kinematics
    //(x,y,z) = ForwardKinematics(q1,q2,q3,q4,q5,q6);
    //Compute the Jacobian
    //J = Jacobian(q1,q2,q3,q4,q5,q6);
    //Compute Inverse:
    //inv_J = damped_Least_squares(q1,q2,q3,q4,q5,q6,Joint_Limits)
    //Compute Update step
    //dq = inv_J*dx;
    //Compute new Target
    //q = q + dq;
    
    stepper1.moveTo(target1);
    stepper2.moveTo(target2);
    stepper3.moveTo(target3);
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
}


//**********INTERRUPTS***********//

//Interrupt 1 Stop tube 1 at the base of the robot:

void collision_interrupt1()
{

  //Count time of the interrupt to deal with switch bouncing
 static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 
 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 200) 
 {
      //Tube 1:
      // If the target is in the backward direction, then ignore it as you are going away from front limit
      if (stepper1.distanceToGo() > 0){
        
      }
      else{
        //if the target is in the forward direction it must be changed to stop going too far forward
        stepper1.stop();
        target1 = -target1;       
      }
 }
  //Remember the time of the interrupt for debouncing
 last_interrupt_time = interrupt_time;
}



//Interrupt 2 stop tube 1 from going backwards, stop tube 2 from going forwards.

void collision_interrupt2()
{

  //Count time of the interrupt to deal with switch bouncing
 static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 
 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 200) 
 {
      //Tube 1:
      // If the target is in the forward direction, then ignore it as you are going away from back limit
      if (stepper1.distanceToGo() < 0 ){
        
      }
      else{
        //if the target is in the backward direction it must be changed to stop going too far back
        stepper1.stop();  
        target1 = -target1;  
      } 
      
      //Tube 2:
      // If the target is in the backward direction, then ignore it as you are going away from front limit
      if (stepper2.distanceToGo() > 0){
        
      }
      else{
        //if the target is in the forward direction it must be changed to stop going too far forward
        stepper2.stop();
        target2 = -target2;   
      }


 }
  //Remember the time of the interrupt for debouncing
 last_interrupt_time = interrupt_time;
}




//Interrupt 3 stop tube 2 from going backwards, stop tube 3 from going forwards.

void collision_interrupt3()
{

  //Count time of the interrupt to deal with switch bouncing
 static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 
 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 200) 
 {
      //Tube 2:
      // If the target is in the forward direction, then ignore it as you are going away from back limit
      if (stepper2.distanceToGo() < 0 ){
        
      }
      else{
        //if the target is in the backward direction it must be changed to stop going too far back
        stepper2.stop();
        target2 = -target2;        
      }
      
      //Tube 3:
      // If the target is in the backward direction, then ignore it as you are going away from front limit
      if (stepper3.distanceToGo() > 0){
        
      }
      else{
        //if the target is in the forward direction it must be changed to stop going too far forward
        stepper3.stop();
        target3 = -target3;        
      }


 }
  //Remember the time of the interrupt for debouncing
 last_interrupt_time = interrupt_time;
}






//Interrupt 4 stop tube 3 from going backwards from the back of the robot


void collision_interrupt4()
{
    //Count time of the interrupt to deal with switch bouncing
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
 
 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 200) 
 {
      //Tube 3:
      // If the target is in the forward direction, then ignore it as you are going away from back limit
      if (stepper3.distanceToGo() < 0 ){
        
      }
      else{
        //if the target is in the backward direction it must be changed to stop going too far back
        stepper3.stop();
        target3 = -target3;        
      }

 }
  //Remember the time of the interrupt for debouncing
 last_interrupt_time = interrupt_time;
}



