/* How to use a flex sensor/resistro - Arduino Tutorial
   Fade an LED with a flex sensor
   More info: http://www.ardumotive.com/how-to-use-a-flex-sensor-en.html
   Dev: Michalis Vasilakis // Date: 9/7/2015 // www.ardumotive.com  */
   

//Constants:
const int indexFin = A0; //pin A0 to read analog input

//Variables:
int value; //save analog value


void setup(){
  
  //pinMode(indexFin, INPUT);
  Serial.begin(9600);       //Begin serial communication
}

void loop(){
  
  value = analogRead(indexFin);         //Read and save analog value from potentiometer
  //Serial.println(value);               //Print value
  // Printing if index finger is open or closed
  /*if (value > 850 && value < 950){
    Serial.println("Neutral");
  }
  else {
    Serial.println("Closed");
  }*/
  value = map(value, 850, 1023, 0, 255);//Map value 0-1023 to 0-255 (PWM)
  Serial.println(value);
  //analogWrite(ledPin, value);          //Send PWM value to led
  delay(100);                          //Small delay
  
}
