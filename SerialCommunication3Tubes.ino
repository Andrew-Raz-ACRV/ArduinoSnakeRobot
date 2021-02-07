// This script allows you to control all the motors based on commands written in the serial monitor
// For instance, write 'q1=1000' in the serial monitor and the first motor will go to 1000 counts
//
// Written by Andrew Razjigaev


#include <AccelStepper.h>

//Global Variables
char recValue;
bool motor = false;
bool read_number = false;
bool process_message = false;
bool read_motor = false;
String inString = "";    // string to hold input
float Value;
int motor_num;
int factor = 1;

// Define a The Prismatic stepper motors
AccelStepper stepper1(AccelStepper::DRIVER, 22, 24);
AccelStepper stepper2(AccelStepper::DRIVER, 30, 32);
AccelStepper stepper3(AccelStepper::DRIVER, 38, 40);

// Define a The Prismatic stepper motors
AccelStepper stepper4(AccelStepper::DRIVER, 48, 50);
AccelStepper stepper5(AccelStepper::DRIVER, 49, 51);
AccelStepper stepper6(AccelStepper::DRIVER, 39, 41);

// Forceps Stepper Motor
AccelStepper stepper7(AccelStepper::DRIVER, 31, 33);

// Define the pin values for the switches, they are interrupt enabled according to this:
// https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
const byte switch1 = 2;
const byte switch2 = 18;
const byte switch3 = 3;
const byte switch4 = 19;

// Initially set the target to be the current start point
double target1 = 0;
double target2 = 0;
double target3 = 0;
double target4 = 0;
double target5 = 0;
double target6 = 0;
double target7 = 0;

void setup()
{
  //Make start position as 0 by default
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
  stepper5.setCurrentPosition(0);
  stepper6.setCurrentPosition(0);
  stepper7.setCurrentPosition(0);
  
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
  pinMode(26, OUTPUT);

  //StepperMotor
  stepper2.setMaxSpeed(1000.0);
  stepper2.setAcceleration(300.0);
  pinMode(34, OUTPUT);

  //StepperMotor
  stepper3.setMaxSpeed(1000.0);
  stepper3.setAcceleration(300.0);
  pinMode(42, OUTPUT);

  //StepperMotor
  stepper4.setMaxSpeed(1000.0);
  stepper4.setAcceleration(300.0);
  pinMode(52, OUTPUT);

  //StepperMotor
  stepper5.setMaxSpeed(1000.0);
  stepper5.setAcceleration(300.0);
  pinMode(53, OUTPUT);

  //StepperMotor
  stepper6.setMaxSpeed(1000.0);
  stepper6.setAcceleration(300.0);
  pinMode(43, OUTPUT);

  //Forceps
  stepper7.setMaxSpeed(1000.0);
  stepper7.setAcceleration(300.0);
  pinMode(35, OUTPUT);
}

void loop()
{
  //Collect message VIA Srial Communication
  if (Serial.available() > 0) {
    recValue = Serial.read();
    Serial.print(recValue);

    if (recValue == 'q') {
      //Serial.print("-Turn on Motor-");
      motor = true;
    }
    else if (recValue == '\n') { //newline end command reset variables/states
      //Serial.print("-Value:");
      Value = factor * inString.toFloat();
      //Serial.println(Value);
      read_number = false;
      inString = "";
      factor = 1;
      if(read_motor == true){
        process_message = false; //next phase motor
        read_motor = false;
      }
      else{
        process_message = true; //next phase motor
      }
      
    }
    else if (recValue == '=') {
      read_number = true;
    }
    else if (recValue == 'r') {
      read_motor = true;
    }
    else if (recValue == '-') {
      factor = -1;
    }


    //motor number from 1 to 7
    if ((motor == true) && (recValue <= 55) && (recValue >= 49)) {
      switch (recValue) {
        case '1':
          motor_num = 1;
          break;
        case '2':
          motor_num = 2;
          break;
        case '3':
          motor_num = 3;
          break;
        case '4':
          motor_num = 4;
          break;
        case '5':
          motor_num = 5;
          break;
        case '6':
          motor_num = 6;
          break;
        case '7':
          motor_num = 7;
          break;
        default:
          Serial.print("-invalid-"); motor_num = 0;
      }
      motor = false;
    }

    if (read_number == true) {
      //read the rest of the numbers
      //Serial.print("-reading-");
      if (isDigit(recValue)) {
        // convert the incoming byte to a char and add it to the string:
        inString += (char)recValue;
      }
    }

      //Now Do what the message commanded: Read
      if (read_motor == true) {
        Serial.println();
        switch (motor_num) {
          case 1:
            Serial.print(-stepper1.currentPosition());
            break;
          case 2:
            Serial.print(-stepper2.currentPosition());
            break;
          case 3:
            Serial.print(-stepper3.currentPosition());
            break;
          case 4:
            Serial.print(stepper4.currentPosition());
            break;
          case 5:
            Serial.print(stepper5.currentPosition());
            break;
          case 6:
            Serial.print(stepper6.currentPosition());
            break;
          case 7:
             Serial.print(stepper7.currentPosition());
            break;
          default:
            Serial.println("-invalid-");
        }
      }
  }

  //Now Do what the message commanded: Move
  if (process_message == true) {
    process_message = false;

    switch (motor_num) {
      case 1:
        target1 = -Value;
        break;
      case 2:
        target2 = -Value;
        break;
      case 3:
        target3 = -Value;
        break;
      case 4:
        target4 = Value;
        break;
      case 5:
        target5 = Value;
        break;
      case 6:
        target6 = Value;
        break;
      case 7:
        target7 = Value;
        break;
      default:
        Serial.println("-invalid-");
    }
  }

  
  //MOVE STEPPER MOTORS
  stepper1.moveTo(target1);
  stepper2.moveTo(target2);
  stepper3.moveTo(target3);
  stepper4.moveTo(target4);
  stepper5.moveTo(target5);
  stepper6.moveTo(target6);
  stepper7.moveTo(target7);
  stepper1.run();
  stepper2.run();
  stepper3.run();
  stepper4.run();
  stepper5.run();
  stepper6.run();
  stepper7.run();
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
    if (stepper1.distanceToGo() > 0) {

    }
    else {
      //if the target is in the forward direction it must be changed to stop going too far forward
      stepper1.stop();
      target1 = stepper1.currentPosition();
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
    if (stepper1.distanceToGo() < 0 ) {

    }
    else {
      //if the target is in the backward direction it must be changed to stop going too far back
      stepper1.stop();
      target1 = stepper1.currentPosition();
    }

    //Tube 2:
    // If the target is in the backward direction, then ignore it as you are going away from front limit
    if (stepper2.distanceToGo() > 0) {

    }
    else {
      //if the target is in the forward direction it must be changed to stop going too far forward
      stepper2.stop();
      target2 = stepper2.currentPosition();
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
    if (stepper2.distanceToGo() < 0 ) {

    }
    else {
      //if the target is in the backward direction it must be changed to stop going too far back
      stepper2.stop();
      target2 = stepper2.currentPosition();
    }

    //Tube 3:
    // If the target is in the backward direction, then ignore it as you are going away from front limit
    if (stepper3.distanceToGo() > 0) {

    }
    else {
      //if the target is in the forward direction it must be changed to stop going too far forward
      stepper3.stop();
      target3 = stepper3.currentPosition();
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
    if (stepper3.distanceToGo() < 0 ) {

    }
    else {
      //if the target is in the backward direction it must be changed to stop going too far back
      stepper3.stop();
      target3 = stepper3.currentPosition();
    }

  }
  //Remember the time of the interrupt for debouncing
  last_interrupt_time = interrupt_time;
}



