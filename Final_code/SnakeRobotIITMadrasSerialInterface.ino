// This script allows you to perform inverse kinematic control of the robot
// That is given a tool point coordinate, the robot will move towards it.
//
// Serial Communication commands:
// q1 = 1000         //This command causes q1 to go to that many steps
// q1r               //This command returns the instantaneous step position of q1
//
// (x35y25z200)       //This gives a target coordinate in mm to the robot. 
//                   //The robot tool point will go to this position
// t                 //This terminates inverse kinematics
// k                 //This kills the robot and resets it to move back t the start
//
// Written by Andrew Razjigaev

#include <AccelStepper.h> //Get from https://github.com/adafruit/AccelStepper
#include <MatrixMath.h> //Get from https://github.com/eecharlie/MatrixMath
#include <math.h> //built in

//Reading Serial Communication Variables
char recValue;
bool motor = false; 
bool read_number = false;
bool process_message = false;
bool read_motor = false;
String inString = "";    // string to hold input
float Value;
int motor_num;
int factor = 1;
//inverse kinematics serial communication commands
bool invk = false; bool terminated = false;
long int timer = 0;
String xString = "";    // string to hold input
String yString = "";    // string to hold input
String zString = "";    // string to hold input
bool reading_coordinate = false; bool process_coordinate = false;
bool x_value = false; float x_sign = 1;
bool y_value = false; float y_sign = 1;
bool z_value = false;

// Define a The Prismatic stepper motors
AccelStepper stepper1(AccelStepper::DRIVER, 22, 24);
AccelStepper stepper2(AccelStepper::DRIVER, 30, 32);
AccelStepper stepper3(AccelStepper::DRIVER, 38, 40);

// Define a The Prismatic stepper motors
AccelStepper stepper4(AccelStepper::DRIVER, 48, 50);
AccelStepper stepper5(AccelStepper::DRIVER, 39, 41);
AccelStepper stepper6(AccelStepper::DRIVER, 49, 51);

// Forceps Stepper Motor
AccelStepper stepper7(AccelStepper::DRIVER, 31, 33);

// Define the pin values for the switches, they are interrupt enabled according to this:
// https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
const byte switch1 = 2; const byte switch2 = 18; const byte switch3 = 3; const byte switch4 = 19;

// Initially set the target to be the current start point
double target1 = 0; double target2 = 0; double target3 = 0; double target4 = 0; 
double target5 = 0; double target6 = 0; double target7 = 0;

//KINEMATICS VARIABLES
const float Pi = 3.141593;

//Jacobian Matrices
float J[3][4];  float invJ[4][3]; float Jt[4][3];
//Other Matrices
float I[4][4];  float D[4][4];  float sqD[4][4];  float inv[4][4];
//transform Matrices
float AB[4][4]; float BC[4][4]; float CD[4][4]; float DE[4][4]; float EF[4][4];
float AC[4][4]; float AD[4][4]; float AE[4][4]; float AF[4][4]; 
//joint value limits
float lowerLim[1][4]; float UpperLim[1][4]; float q[1][4];
//Differentials:
float dX[3][1]; float dq[4][1];
//Speed limit
float mag; float speed_limit = 100; //mm

//Constnt radius of tube curvature
const float r = 175;

//Toolpoint
float x; float y; float z;
//Desired Point 
float X; float Y; float Z;

//Calibration factor
float steppmm = 1250; //step per mm
float stepprad = 2750/Pi; // step per radian

//Initial joint values
float q1 = 0; float q2 = 0; float q3 = 0; float q4 = 0; float q5 = 0; float q6 = 0;

//******************************SETUP**********************************************************//
void setup()
{
  //Start serial communication
  Serial.begin(9600);
  Serial.println("The Snake Robot IIT Madras");
  Serial.println("this program was written by Andrew Razjigaev on the 8/06/2018");
  Serial.println();
  Serial.println("Give motor commands like this: q1=1000");
  Serial.println("Read motor values like this: q1r");
  Serial.println("Give a toolpoint target like this: (x10y-10z100)");
  Serial.println("This robot will perform the inverse kinematics provided that the joint limits are measured");
  Serial.println("type t to terminate the inverse kinematics or stop");
  Serial.println("Or use k to reset the robot completely to its initial position");
  Serial.println("type f to read the tool position");
  Serial.println();
  
  //Joint limit set up
  lowerLim[0][0] = 0; //q1 lowerlimit
  lowerLim[0][1] = 0; //q2 lowerlimit
  lowerLim[0][2] = 0; //q3 lowerlimit
  lowerLim[0][3] = -4*Pi; //q5 lowerlimit

  UpperLim[0][0] = 61159/steppmm; //q1 Upperlimit in mm
  UpperLim[0][1] = 50685/steppmm; //q2 Upperlimit in mm
  UpperLim[0][2] = 55135/steppmm; //q3 Upperlimit in mm
  UpperLim[0][3] = 4*Pi; //q5 Upperlimit in rad  
  //Joint vector q
  q[0][0] = q1; q[0][1] = q2; q[0][2] = q3; q[0][3] = q5;
  
  //Make start position as 0 by default
  stepper1.setCurrentPosition(0); stepper2.setCurrentPosition(0); stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0); stepper5.setCurrentPosition(0); stepper6.setCurrentPosition(0);
  stepper7.setCurrentPosition(0);

  //Switch interrupt initialisation
  pinMode(switch1, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(switch1), collision_interrupt1, FALLING);
  pinMode(switch2, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(switch2), collision_interrupt2, FALLING);
  pinMode(switch3, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(switch3), collision_interrupt3, FALLING);
  pinMode(switch4, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(switch4), collision_interrupt4, FALLING);

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
  pinMode(43, OUTPUT);

  //StepperMotor
  stepper6.setMaxSpeed(1000.0);
  stepper6.setAcceleration(300.0);
  pinMode(53, OUTPUT);

  //Forceps
  stepper7.setMaxSpeed(1000.0);
  stepper7.setAcceleration(300.0);
  pinMode(35, OUTPUT);
}

//*******************************MAIN LOOP*************************************//
void loop()
{
  //*************************************************************************************************************//
  //***********************************************SERIAL COMMUNICATION******************************************//
  //*************************************************************************************************************//
  //Collect message VIA Serial Communication
  if (Serial.available() > 0) {
    recValue = Serial.read();
    Serial.print(recValue);

    if (recValue == 'q') {
      //Serial.print("-Turn on Motor-");
      motor = true;
    }
    else if (recValue == '\n') { //newline end command reset variables/states
      Value = factor * inString.toFloat();
      read_number = false;
      inString = "";
      factor = 1;
      terminated = false;

      if(reading_coordinate == true){
        process_coordinate = true; //next phase
        z_value = false;
        reading_coordinate = false;
      }
      
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
      if(x_value==true){
        x_sign = -1;
      }
      else if(y_value==true){
        y_sign = -1;
      }
      else if(z_value==true){
        Serial.println("Tube Cannot move to -Z");
      }
    } 
    //Inverse kinematics
    else if (recValue == '(') {
      reading_coordinate = true;
    }
    else if (recValue == ')') {
      z_value = false;
    }
    else if (recValue == 'x') {
      x_value = true;
    }
    else if (recValue == 'y') {
      y_value = true;
      x_value = false;
    }
    else if (recValue == 'z') {
      z_value = true;
      y_value = false;
    }
    else if (recValue == 't'){
      invk = false;
      motor = false;
      read_motor = false;
      reading_coordinate = false;
      read_number = false;
      process_message = false;
      terminated = true;
    }
    else if (recValue == 'k'){
      //Stop all the processes
      invk = false;
      motor = false;
      read_motor = false;
      reading_coordinate = false;
      read_number = false;
      process_message = false;
      
      Serial.println();
      Serial.println("Resetting Robot arm: revolute first");
      target7 = 0;
      target4 = 0; target5 = 0; target6 = 0;
      Serial.println("Resetting Robot arm: prismatic joints");
      delay(1000);
      target3 = 0; delay(500);
      target2 = 0; delay(500);
      target1 = 0;  
    }
    else if (recValue == 'f'){
      //Read Joints remember in practise q2 is relative to q1 and q3 to q2
      q1 = (-stepper1.currentPosition())/steppmm;
      q2 = (-stepper1.currentPosition() - stepper2.currentPosition() )/steppmm;
      q3 = (-stepper1.currentPosition() - stepper2.currentPosition() - stepper3.currentPosition())/steppmm;
      q4 = stepper4.currentPosition()/stepprad;
      q5 = stepper5.currentPosition()/stepprad;
      q6 = stepper6.currentPosition()/stepprad;
      
      //Forward kinematics
      forwardkinematics(q1, q2, q3, q4, q5, q6);
      Serial.println();
      Serial.print("X= ");
      Serial.print(x);
      Serial.print(" Y= ");
      Serial.print(y); 
      Serial.print(" Z= ");
      Serial.print(z);  
    }

//*************Target coordinate read logic****************//
    if(reading_coordinate == true){
      if(x_value == true){
        if(isDigit(recValue)) {
          // convert the incoming byte to a char and add it to the string:
          xString += (char)recValue;
        }
      }
      if(y_value == true){
        if(isDigit(recValue)) {
          // convert the incoming byte to a char and add it to the string:
          yString += (char)recValue;
        }
      }
      if(z_value == true){
        if(isDigit(recValue)) {
          // convert the incoming byte to a char and add it to the string:
          zString += (char)recValue;
        }
      }
    }

    if(process_coordinate == true){
      X = x_sign*xString.toFloat(); xString = "";
      Y = y_sign*yString.toFloat(); yString = "";
      Z = zString.toFloat(); zString = "";
      x_sign = 1; y_sign = 1; //maintain consistency
      invk = true;
      process_coordinate = false;
      timer = 0;
      Serial.println("Beginning Inverse Kinematics");
    }


//**************individual motor command logic*************//
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
    }
  }
//*************************************************************************************************************//
//*****************************************INVERSE KINEMATICS CONTROL******************************************//
//*************************************************************************************************************//
if((invk == true)&&((timer % 250)==0)){
  
    //Read Joints remember in practise q2 is relative to q1 and q3 to q2
    q1 = (-stepper1.currentPosition())/steppmm;
    q2 = (-stepper1.currentPosition() - stepper2.currentPosition() )/steppmm;
    q3 = (-stepper1.currentPosition() - stepper2.currentPosition() - stepper3.currentPosition())/steppmm;
    q4 = stepper4.currentPosition()/stepprad;
    q5 = stepper5.currentPosition()/stepprad;
    q6 = stepper6.currentPosition()/stepprad;
    //Joint vector q
    q[0][0] = q1; q[0][1] = q2; q[0][2] = q3; q[0][3] = q5;  
    
    //Forward kinematics
    forwardkinematics(q1, q2, q3, q4, q5, q6);
    Serial.print("X= ");
    Serial.print(x);
    Serial.print(" Y= ");
    Serial.print(y); 
    Serial.print(" Z= ");
    Serial.print(z);  
    Serial.println();

    //Compute Error:
    dX[0][0] = X - x;
    dX[1][0] = Y - y;
    dX[2][0] = Z - z;

    //logic to stop at goal based on magnitude
     mag = sqrt(sq(dX[0][0]) + sq(dX[1][0]) + sq(dX[2][0]));
     
    if(mag < 5){ // if within 5mm radius
      Serial.println("Inverse Kinematics Calculation complete within:");
      Serial.print(timer);
      Serial.print(" iterations");
      Serial.println();
      //Stop motors etc;
      invk = false; motor = false;
      read_motor = false; reading_coordinate = false;
      read_number = false; process_message = false;
      terminated = true;
    }

    //Regulate speed 
    if(mag > speed_limit){
      for(int ii = 0; ii < 3; ii++){
        dX[ii][0] = speed_limit*(dX[ii][0])/mag;        
      }
    } //its too slow for this
    
    //Compute Jacobian
    computeJacobian(q2, q3, q5);
    
    //Compute inverse invJ
    dampedLeastSquares();

    //Compute update step
    //dq = invJ*dX
    Matrix.Multiply((float*)invJ, (float*)dX, 4, 3, 1, (float*)dq);

    //Update step:
    q1 = q1 + dq[0][0];
    q2 = q2 + dq[1][0];
    q3 = q3 + dq[2][0];
    q5 = q5 + dq[3][0];

    //Joint vector q
    q[0][0] = q1; q[0][1] = q2; q[0][2] = q3; q[0][3] = q5;  

    //Enforce Joint Limits
    enforceJointLimit();

    //Convert to steps for the motor targets remember in practise q2 is relative to q1 and q3 to q2
    target1 = -q1*steppmm;
    target2 = target1 - q2*steppmm;
    target3 = target2 - q3*steppmm;
    target5 = q5*stepprad;
    
    //Timer for inverse kinematics to end if stuck
    timer++;

    if(timer>100000){
      invk = false;
      Serial.println("Failed to reach target, maybe close to it, timed out");
    }
    
}
else if(invk == true){
  timer++;
}


if(terminated == true){
      stepper1.stop();  target1 = stepper1.currentPosition(); 
      stepper2.stop();  target2 = stepper2.currentPosition(); 
      stepper3.stop();  target3 = stepper3.currentPosition(); 
      stepper5.stop();  target5 = stepper5.currentPosition(); 
      terminated = false;
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

//*************************************************************************************************************//
//***********************************************KINEMATICS FUNCTIONS******************************************//
//*************************************************************************************************************//

void enforceJointLimit(){
  //Iterate and check limits
  for(int ii = 0; ii < 3; ii++){
    if(q[0][ii] < lowerLim[0][ii]){
      q[0][ii] = lowerLim[0][ii];
    }
    else if(q[0][ii] > UpperLim[0][ii]){
      q[0][ii] = UpperLim[0][ii];
    }
  }
  //Set value
  q1 = q[0][0]; q2 = q[0][1]; q3 = q[0][2];
}

//Basic functions:
void identity44(float I[4][4]){
  for(int ii = 0; ii < 4; ii++){
    for(int jj = 0; jj < 4; jj++){
      if(ii==jj){
        I[ii][jj] = 1;
      }
      else{
        I[ii][jj] = 0;
      }
    }   
  }
}

void TranslationTransform(float matrix[4][4],float x, float y, float z){
  matrix[0][3] = x; matrix[1][3] = y; matrix[2][3] = z;
}

void RotationX(float matrix[4][4], float theta){
  matrix[1][1] = cos(theta); matrix[1][2] = sin(theta); 
  matrix[2][1] = -sin(theta); matrix[2][2] = cos(theta); 
}

void RotationZ(float matrix[4][4], float theta){
  matrix[0][0] = cos(theta); matrix[0][1] = -sin(theta); 
  matrix[1][0] = sin(theta); matrix[1][1] = cos(theta); 
}

void computeJacobian(float q2, float q3, float q5){
  J[0][0] = 0; J[0][1] = -sin(q2/r)*sin(q5)-(q3*cos(q2/r)*sin(q5))/r; J[0][2] = -sin(q2/r)*sin(q5); J[0][3] = r*cos(q5)*(cos(q2/r)-1)-q3*sin(q2/r)*cos(q5); 
  J[1][0] = 0; J[1][1] =  sin(q2/r)*cos(q5)+(q3*cos(q2/r)*cos(q5))/r; J[1][2] =  sin(q2/r)*cos(q5); J[1][3] = r*sin(q5)*(cos(q2/r)-1)-q3*sin(q2/r)*sin(q5); 
  J[2][0] = 1; J[2][1] = cos(q2/r)-(q3*sin(q2/r))/r; J[2][2] = cos(q2/r); J[2][3] = 0;
}

void forwardkinematics(float q1, float q2, float q3, float q4, float q5, float q6){
  //Construct homogeneous transforms
  identity44(AB); //base to tube 1
  TranslationTransform(AB,0, 0, q1);

  identity44(BC); //tube 1 to q5 rotation
  RotationZ(BC, q5);

  identity44(CD); //q5 rotation to tube 2 tip
  RotationX(CD,q2/r);  
  TranslationTransform(CD,0, r*(1-cos(q2/r)), r*sin(q2/r));

  identity44(DE); //q2 to tube 3
  TranslationTransform(DE,0, 0, q3);

  identity44(EF); //tool rotation
  RotationZ(EF, q6);

  //Cascade transforms from base to tool point
  Matrix.Multiply((float*)AB, (float*)BC, 4, 4, 4, (float*)AC);
  Matrix.Multiply((float*)AC, (float*)CD, 4, 4, 4, (float*)AD);
  Matrix.Multiply((float*)AD, (float*)DE, 4, 4, 4, (float*)AE); 
  Matrix.Multiply((float*)AE, (float*)EF, 4, 4, 4, (float*)AF);

  //Output Forward Kinematics
  x = AF[0][3]; y = AF[1][3]; z = AF[2][3];
}

void dampedLeastSquares(){

  //damping matrix D
  identity44(D);
  for(int ii = 0; ii<4; ii++){
    D[ii][ii] = sq((2*q[0][ii] - UpperLim[0][ii] - lowerLim[0][ii])/(UpperLim[0][ii] - lowerLim[0][ii])) + 1;
  }
  //Matrix.Print((float*)D, 4, 4, "D");
  
  //Find inverse formula:  inv_J = inv(J'*J + D^2)*J'
  Matrix.Multiply((float*)D, (float*)D, 4, 4, 4, (float*)sqD);//square D
  //Matrix.Print((float*)sqD, 4, 4, "D^2");
  Matrix.Transpose((float*)J, 3, 4, (float*)Jt); //J' 4 by 3
  //Matrix.Print((float*)Jt, 4, 3, "Jt");
  Matrix.Multiply((float*)Jt, (float*)J, 4, 3, 4, (float*)inv);//J'*J
  //Matrix.Print((float*)inv, 4, 4, "Jt*J");
  Matrix.Add((float*)inv, (float*)sqD, 4, 4, (float*)inv); // J'J + D^2
  //Matrix.Print((float*)inv, 4, 4, "Jt*J + D^2");
  Matrix.Invert((float*)inv, 4);
  //Matrix.Print((float*)inv, 4, 4, "inv(Jt*J + D^2)");
  Matrix.Multiply((float*)inv, (float*)Jt, 4, 4, 3, (float*)invJ);//answer
}

//*************************************************************************************************************//
//*************************************************************************************************************//
//*******************************************************INTERRUPTS********************************************//
//*************************************************************************************************************//
//*************************************************************************************************************//

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



