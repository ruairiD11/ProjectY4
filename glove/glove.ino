/* Remote-Robot  
 *  Ruairi Doherty Final Year Project GMIT 2019/20
 *  This code is for the glove module of my project
*/

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
===============================================
*/

#include "I2Cdev.h"
#include "MPU6050.h"
#include <Servo.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//Initialise analog pin for each finger
const int INDEX = A0;
const int MIDDLE = A1;
const int THIRD = A2;

MPU6050 accelgyro;
Servo roll, gripper;
int16_t ax, ay, az;
int16_t gx, gy, gz;

int indexValue; // value read from index finger
int middleValue; // value read from middle finger
int thirdValue; // value read from third finger
int avg, sum; // sum and average of the three values
int rollVal, gripperVal; // values to be written to each servo

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  
  Serial.begin(38400);
  roll.attach(21);
  roll.write(0);
  gripper.attach(17);
  gripper.write(0);

  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop() {
  indexValue = analogRead(INDEX);
  indexValue = constrain(indexValue, 530, 800); 
  
  middleValue = analogRead(MIDDLE);
  //Middle flex sensor can jump to > 1000 when the finger is open sometimes
  //So if the value goes above 1000, I set it to the minimum value which is 550 for this sensor
  if(middleValue > 1000){
    middleValue = 550;
  }
  middleValue = constrain(middleValue, 550, 730);
  
  thirdValue = analogRead(THIRD);
  thirdValue = constrain(thirdValue, 540, 730);

  // read raw accel/gyro measurements from device
  // All I am using at the moment is ay for roll
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //Calculating the average of these three values
  // The average is the value that will be sent to the Servo Motor
  sum = indexValue + middleValue + thirdValue;
  avg = sum/3;

  gripperVal = map(avg, 600, 750, 0, 180);
  gripperVal = constrain(gripperVal, 0, 180);
  
  rollVal = ay;
  rollVal = map(rollVal, -17000, 16000, 0, 180);
  rollVal = constrain(rollVal, 0, 180);
  
  roll.write(rollVal);
  gripper.write(gripperVal);

  /*sum = indexValue + thirdValue;
  avg = sum/2;

  gripperVal = avg;
  gripperVal = map(gripperVal, 550, 780, 0, 180);
  gripperVal = constrain(gripperVal, 0, 180);
  

  rollVal = ay;
  rollVal = map(rollVal, -17000, 16000, 0, 180);
  rollVal = constrain(rollVal, 0, 180);
  
  roll.write(rollVal);
  gripper.write(gripperVal);*/
  
  Serial.print("Roll: ");
  Serial.print(rollVal);
  Serial.print("\tGripper: ");
  Serial.println(gripperVal);

  delay(100);
  
}
