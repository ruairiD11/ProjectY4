/* Remote-Robot  
 *  Ruairi Doherty Final Year Project GMIT 2019/20
 *  This code is for the glove module of my project
*/

/*
 * References:
 * EEEnthusiast
 * "Arduino Accelerometer & Gyroscope Tutorial MPU-6050 6DOF Module"
 * https://www.youtube.com/watch?v=M9lZ5Qy5S2s&list=LLvh5ROdXsPMvtfBtygank5g
 * 
 */

#include "Wire.h"
#include <Servo.h>

//Initialise analog pin for each finger
const int INDEX = A0;
const int MIDDLE = A1;
const int THIRD = A2;

Servo roll, gripper; //Initialising servo for the gripper and roll
int16_t ax, ay, az; //x y and z read from accelerometer, only using y value at the moment

int indexValue; // value read from index finger
int middleValue; // value read from middle finger
int thirdValue; // value read from third finger
int avg, sum; // sum and average of the three values
int rollVal, gripperVal; // values to be written to each servo

void setup() {
  Serial.begin(9600);
  roll.attach(10);
  roll.write(0);
  gripper.attach(17);
  gripper.write(0);
  Wire.begin(); //Initializing the I2C communication for the MPU
  setupMPU();
}

void loop() {
  readAccelRegisters();
  readFlexSensors();
  servoWrite();
  printData();
  delay(100);
}

void setupMPU() {
  //Establish communication with the MPU 6050
  //Setup up the registers I will be using to read data from MPU 6050
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (AD0=0 - 0b1101000)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (PWR_MGMT_1)
  Wire.write(0b00000000); //Setting SLEEP register to 0 (Bit 6)
  Wire.endTransmission();

  //Setting up Accelerometer
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C); // (ACCEL_CONFIG) register
  Wire.write(0x00000000); // Setting the accel full scale range to +/- 2g
  Wire.endTransmission();
}

void readAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of MPU
  Wire.write(0x3B); // Starting request to register for Accel readings
  Wire.endTransmission(); 
  Wire.requestFrom(0b1101000,6); // Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  ax = Wire.read()<<8|Wire.read(); //First two bytes stored in ax
  ay = Wire.read()<<8|Wire.read(); //Middle two bytes stored in ay
  az = Wire.read()<<8|Wire.read(); //Last two bytes stored in az
}

void readFlexSensors() {
  indexValue = analogRead(INDEX);
  indexValue = constrain(indexValue, 530, 800); 
  
  middleValue = analogRead(MIDDLE);
  if(middleValue > 1000){
    middleValue = 550;
  }
  middleValue = constrain(middleValue, 550, 730);
  
  thirdValue = analogRead(THIRD);
  thirdValue = constrain(thirdValue, 540, 730);

  //Calculating the average of these three values
  // The average is the value that will be sent to the Servo Motor
  sum = indexValue + middleValue + thirdValue;
  avg = sum/3;
}

void servoWrite() {
  gripperVal = map(avg, 600, 750, 0, 180);
  gripperVal = constrain(gripperVal, 0, 180);
  
  rollVal = ay;
  rollVal = map(rollVal, -17000, 16000, 0, 180);
  rollVal = constrain(rollVal, 0, 180);
  
  roll.write(rollVal);
  gripper.write(gripperVal);
}

void printData() {
  Serial.print("Roll: ");
  Serial.print(rollVal);
  Serial.print("\tGripper: ");
  Serial.println(gripperVal);
}
