// MultiStepperBlocking
// Andrew

#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper1(AccelStepper::DRIVER, 2, 3);
AccelStepper stepper2(AccelStepper::DRIVER, 5, 6);
AccelStepper stepper3(AccelStepper::DRIVER, 8, 9);

AccelStepper stepper4(AccelStepper::DRIVER, 36, 38);
AccelStepper stepper5(AccelStepper::DRIVER, 30, 32);
AccelStepper stepper6(AccelStepper::DRIVER, 31, 33);

void setup()
{  
    Serial.begin(9600);
    
    stepper1.setMaxSpeed(1000.0);
    stepper1.setAcceleration(300.0);
    pinMode(4,OUTPUT);
    
    stepper2.setMaxSpeed(1000.0);
    stepper2.setAcceleration(300.0);
    pinMode(7,OUTPUT);

    stepper3.setMaxSpeed(1000.0);
    stepper3.setAcceleration(300.0);
    pinMode(10,OUTPUT);

    stepper4.setMaxSpeed(500.0);
    stepper4.setAcceleration(300.0);
    pinMode(40,OUTPUT);

    stepper5.setMaxSpeed(500.0);
    stepper5.setAcceleration(300.0);
    pinMode(34,OUTPUT);

    stepper6.setMaxSpeed(500.0);
    stepper6.setAcceleration(300.0);
    pinMode(35,OUTPUT);
}

void loop()
{    

//prismatic Joints
      Serial.println("moving tube 1 forward by 400"); //6400
      stepper1.runToNewPosition(-400*32);
      delay(1000);
      
      Serial.println("moving tube 1 backward to 0");
      stepper1.runToNewPosition(0);
      delay(1000);

      Serial.println("moving tube 2 forward by 400");
      stepper2.runToNewPosition(-400*32);
      delay(1000);

      Serial.println("moving tube 2 backward to 0");
      stepper2.runToNewPosition(0);
      delay(1000);

      Serial.println("moving tube 3 forward by 400");
      stepper3.runToNewPosition(-400*32);
      delay(1000);

      Serial.println("moving tube 3 backward to 0");
      stepper3.runToNewPosition(0);
      delay(1000);

//Revolute joints
      Serial.println("rotating tube 1 forward by 400"); //6400
      stepper4.runToNewPosition(-400*32);
      delay(1000);
      
      Serial.println("rotating tube 1 backward to 0");
      stepper4.runToNewPosition(0);
      delay(1000);

      Serial.println("rotating tube 2 forward by 400");
      stepper5.runToNewPosition(-400*32);
      delay(1000);

      Serial.println("rotating tube 2 backward to 0");
      stepper5.runToNewPosition(0);
      delay(1000);

      Serial.println("rotating tube 3 forward by 400");
      stepper6.runToNewPosition(-400*32);
      delay(1000);

      Serial.println("rotating tube 3 backward to 0");
      stepper6.runToNewPosition(0);
      delay(1000);    
    
    //Disable the motors
    digitalWrite(4,LOW);
    digitalWrite(7,LOW);
    digitalWrite(10,LOW);

    digitalWrite(40,LOW);
    digitalWrite(34,LOW);
    digitalWrite(35,LOW);
} 



//void MoveArm(AccelStepper m1, AccelStepper m2, int p1, int p2)
//{ 
//    //Command stepper motors to move to new targets p1, p2
//    m1.moveTo(p1);
//    m2.moveTo(p2);
//
//    //Block until the stepper motors are there
//    bool running = true;
//    while(running == true){
//      m1.run();
//      m2.run();
//      //condition to unblock
//      if ((m1.distanceToGo() == 0)&&(m2.distanceToGo() == 0)){
//        running = false;
//      }
//    }
//}
       
//    Serial.println("Running Stepper 1 & 2 to Position 0");
//    MoveArm(stepper1,stepper2,0,0);
//    delay(1000);
//    
//    Serial.println("Running Stepper 1 & 2 to Position 500");
//    MoveArm(stepper1,stepper2,500,500);
//    delay(1000);
//    
//    Serial.println("Running Stepper 1 & 2 to Position 100");
//    MoveArm(stepper1,stepper2,100,100);
//    delay(1000);
//    
//    Serial.println("Running Stepper 1 & 2 to Position 120");
//    MoveArm(stepper1,stepper2,120,120);
//    delay(1000);
