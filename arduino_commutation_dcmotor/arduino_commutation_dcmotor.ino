int motorPin1 = 3;
int motorPin2 = 4;
int motorSpeedPin = 5;

int inputValue = 0;
int outputValue = 0;

int ledPin = 13;

int runPin = 2;
int runState = 0;
int daqSpeedPin = 1;

int heartbeatPin = 8;
int heartbeatState = LOW;
int heartbeatPrevState = HIGH;
int heartbeatMissCount = 0;
int heartbeatMissMax = 250;
int heartbeatActive = 1;

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorSpeedPin, OUTPUT);

  pinMode(heartbeatPin, INPUT);
  
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin,LOW);
  
  pinMode(runPin,INPUT);
  analogWrite(motorSpeedPin, 0); 
}

void loop() {
  // Heartbeat check
  heartbeatState = digitalRead(heartbeatPin);
  if (heartbeatState  == heartbeatPrevState ) {
    heartbeatMissCount += 1;
  }
  else {
    heartbeatPrevState = heartbeatState;
    heartbeatMissCount = 0;
  }
  if (heartbeatMissCount > heartbeatMissMax) {
    heartbeatActive = 0;
  }

  inputValue = analogRead(daqSpeedPin);
  inputValue = (inputValue - 512)/2;
   
  runState = digitalRead(runPin);
  delay(1);

  if (runState ==  LOW) {
    digitalWrite(ledPin,LOW);

    if (heartbeatActive == 1) {
      if (inputValue > 100) {
        outputValue = min(inputValue,255);
        digitalWrite(motorPin1, 1);    
        digitalWrite(motorPin2, 0);
        }
      else if (inputValue < -100) {
        outputValue = -max(inputValue,-255);
        digitalWrite(motorPin1, 0);    
        digitalWrite(motorPin2, 1);
      }
      else {
        digitalWrite(motorPin1, 0); 
        digitalWrite(motorPin2, 0);
      }
      
      analogWrite(motorSpeedPin, outputValue);
    }
    else {
      analogWrite(motorSpeedPin, 0);
    }
  }
  else {
    digitalWrite(ledPin,HIGH);

    heartbeatActive = 1;
    heartbeatMissCount = 0;
    
    digitalWrite(motorPin1, 1);    
    digitalWrite(motorPin2, 0);
    analogWrite(motorSpeedPin, 195); 
  }
}
