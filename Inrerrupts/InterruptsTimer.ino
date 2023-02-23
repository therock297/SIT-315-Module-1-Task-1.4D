int led1 = 13;                // the pin that the LED is attached to
int led2 = 8;
int led3 = 12;
int led4 = 11;
int sensor1 = 2;              // the pin that the sensor is attached to
int sensor2 = 3;
int sensor3 = 4;
volatile int state1 = LOW;    // by default, no motion detected
volatile int state2 = LOW;
volatile int state3 = LOW;

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR (PCINT2_vect) // handle pin change interrupt for D8 to D13 here
 {    
     motionDetected3();
 }

void setup() {
  
  pinMode(led1, OUTPUT);      // initialize LED as an output
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  
  pinMode(sensor1, INPUT);    // initialize sensor as an input
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT); 
  
  attachInterrupt(digitalPinToInterrupt(sensor1), motionDetected1, HIGH);
  attachInterrupt(digitalPinToInterrupt(sensor2), motionDetected2, HIGH);
 // attachInterrupt(digitalPinToInterrupt(sensor3), motionDetected3, HIGH);
  // attach interrupt to sensor pin, trigger on rising edge (HIGH)
  Serial.begin(9600);        // initialize serial
startTimer(0.5); // Start timer with frequency of 0.5 Hz (blinks every 2 seconds)

  // enable interrupt for pin...
  pciSetup(4);
}

void loop(){
  // do other stuff here, if needed
}

void startTimer(double timerFrequency){
  noInterrupts();

  // Calculate the value for OCR1A based on the timer frequency
  uint16_t ocrValue = (uint16_t)(F_CPU / 1024.0 / timerFrequency - 1);

  // Set the Timer1 registers for CTC mode and set the OCR1A value
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = ocrValue;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);

  interrupts();
}
ISR(TIMER1_COMPA_vect){
   digitalWrite(led4, digitalRead(led4) ^ 1);
}


void motionDetected1() {
  digitalWrite(led1, !state1);   // toggle LED
  state1 = !state1;              // update variable state
  if (state1 == HIGH) {
    Serial.println("Motion 1 detected!"); 
  } 
  else {
    Serial.println("Motion 1 stopped!");
  }
}
void motionDetected2() {
  digitalWrite(led2, !state2);   // toggle LED
  state2 = !state2;              // update variable state
  if (state2 == HIGH) {
    Serial.println("Motion 2 detected!"); 
  } 
  else {
    Serial.println("Motion 2 stopped!");
  }
  
}
  
  void motionDetected3() {
  digitalWrite(led3, !state3);   // toggle LED
  state3 = !state3;              // update variable state
  if (state3 == HIGH) {
    Serial.println("Motion 3 detected!"); 
  } 
  else {
    Serial.println("Motion 3 stopped!");
  }
}
