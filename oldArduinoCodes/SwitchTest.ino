//Switch Bump test
// Andrew

//pin value for the switch signal
    int switch1 = 48;
    int switch2 = 50;
    int switch3 = 52;
    int switch4 = 46;
    int switch5 = 44;
    int switch6 = 42;
    
//intialise input signal
    int val1 = 0;
    int val2 = 0;
    int val3 = 0;
    int val4 = 0;
    int val5 = 0;
    int val6 = 0;


    
void setup() {
  
    Serial.begin(9600);
    
    //initialise input Switches
    pinMode(switch1,INPUT);
    pinMode(switch2,INPUT);
    pinMode(switch3,INPUT);
    pinMode(switch4,INPUT);
    pinMode(switch5,INPUT);
    pinMode(switch6,INPUT);
    
    
}

void detect_collision(int Switch, int val) 
{
    if (val == LOW){
      Serial.print("Switch ");
      Serial.print(Switch);
      Serial.println(" bump occured");
      
      while (val == LOW){  
        val = digitalRead(Switch); 
        delay(200);        
      }
    } 
}

void loop() {
  
    //read swith values
    val1 = digitalRead(switch1);
    val2 = digitalRead(switch2);
    val3 = digitalRead(switch3);
    val4 = digitalRead(switch4);
    val5 = digitalRead(switch5);
    val6 = digitalRead(switch6);

  //if statement for reading a switch signal once (about 3 times due to bouncing)
    detect_collision(switch1,val1);
    detect_collision(switch2,val2);
    detect_collision(switch3,val3);
    detect_collision(switch4,val4);
    detect_collision(switch5,val5);
    detect_collision(switch6,val6);
    
}
