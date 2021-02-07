// Tube limit Test
// Sends each prismatic joint tube to positive and negative limits

#include <AccelStepper.h>

// Define a The Prismatic stepper motors
AccelStepper stepper1(AccelStepper::DRIVER, 2, 3);
AccelStepper stepper2(AccelStepper::DRIVER, 5, 6);
AccelStepper stepper3(AccelStepper::DRIVER, 8, 9);

// Define pin value for the limit switch signal, 
// initialise the limit value
    int switch1 = 52;    int val1 = 0;
    int switch2 = 50;    int val2 = 0;
    int switch3 = 48;    int val3 = 0;
    int switch4 = 49;    int val4 = 0;
    int switch5 = 51;    int val5 = 0;
    int switch6 = 53;    int val6 = 0;

void initialise_Stepper(AccelStepper stepper, int Speed, int acceleration, int enablepin){
    stepper.setMaxSpeed(Speed);
    stepper.setAcceleration(acceleration);
    pinMode(enablepin,OUTPUT);    
}

void setup() {
    Serial.begin(9600);

    stepper1.setMaxSpeed(10000.0);
    stepper1.setAcceleration(500.0);
    pinMode(4,OUTPUT);

    //initialise Limit switches
    pinMode(switch1,INPUT);
    pinMode(switch2,INPUT);
    pinMode(switch3,INPUT);
    pinMode(switch4,INPUT);
    pinMode(switch5,INPUT);
    pinMode(switch6,INPUT);
}


void run_to_limit_and_back(AccelStepper stepper, int Switch, int Direction){
    //Run to the joint limit
    bool collision = false;
    //int infinite_target = 400*32*Direction;
    bool val = HIGH;
 
    while(collision==false){
      Serial.println("Moving");
      stepper.move(10000*Direction);
      stepper.setSpeed(1000);

      for(int ii = 0; ii<10; ii++){
        stepper.run();
      }
      
      //increment the target so that it is impossible to catch up to it
      //infinite_target = infinite_target + infinite_target;
      
      
      //read swith values
      val = digitalRead(Switch);

      //Check joint limit
      if (val == LOW){
        Serial.print("Collision occured going back");
        stepper.stop();
        //Go back to start
        stepper.runToNewPosition(0);
        collision = true;
      } 
    }


}


void loop() {

    stepper1.runToNewPosition(400*32);
    delay(10000);
    run_to_limit_and_back(stepper1, switch1, 1);

    //Disable the motors
    digitalWrite(4,LOW);
    digitalWrite(7,LOW);
    digitalWrite(10,LOW);

    delay(1000000);
}
