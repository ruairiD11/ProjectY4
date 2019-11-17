/* Remote-Robot  
 *  Ruairi Doherty Final Year Project GMIT 2019/20
 *  This code is for the glove module of my project
*/

//Initialise analog pin for each finger
const int INDEX = A0;
const int MIDDLE = A1;
const int THIRD = A2;

int indexValue; // value read from index finger
int middleValue; // value read from middle finger
int thirdValue; // value read from third finger
int avg, sum; // sum and average of the three values

void setup() {
  Serial.begin(9600);
}

void loop() {
  indexValue = analogRead(INDEX); 
  middleValue = analogRead(MIDDLE);
  thirdValue = analogRead(THIRD);

  //Calculating the average of these three values
  // The average is the value that will be sent to the Servo Motor
  sum = indexValue + middleValue + thirdValue;
  avg = sum/3;

  Serial.println(avg);
  delay(100);
  
}
