const int switchPin = 2;
const int motorPin = 9;

int switchState = 0;

void setup() {
  pinMode(motorPin, OUTPUT);
  pinMode(switchPin, INPUT);
}

void loop() {
  switchState = digitalRead(switchPin);

  if(switchState == HIGH){
    analogWrite(motorPin, 255);
    delay(800);
    digitalWrite(motorPin, LOW);
  }
  else{
    digitalWrite(motorPin, LOW);
  }
}
