/* Using the Arduino UNO 
   Mapping the analog input of the flex sensor to the servo motor 
*/

#include <Servo.h>

Servo servo;
const int indexFin = A0;
int value; // value read from flex sensor

void setup() {
  servo.attach(9);
  servo.write(0);
  Serial.begin(9600);
}

void loop() {
  value = analogRead(indexFin);
  value = map(value, 550, 780, 0, 180);//Map value 0-1023 to 0-255 (PWM)
  servo.write(value);
  Serial.println(value);
  delay(100);
  
}
