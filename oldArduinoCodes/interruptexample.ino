// Tube limit Test
// Sends the first prismatic joint tube to its negative limit

const byte interruptPin = 3;
const byte interruptPin2 = 18;
volatile byte state = HIGH;



void setup() {
  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), my_interrupt_handler, FALLING);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), my_interrupt_handler, FALLING);

}

void loop() {
  Serial.println("Working");
  delay(1000);
}

void my_interrupt_handler()
{
 static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 200) 
 {
     Serial.println("Switch Interrupt");
     delay(2000);
 }
 last_interrupt_time = interrupt_time;
}

