/*
 * Library: https://github.com/bolderflight/MPU9250
Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/*
 * Updated by Ahmad Shamshiri on July 09, 2018 for Robojax.com
 * in Ajax, Ontario, Canada
 * watch instrucion video for this code: 
For this sketch you need to connect:
VCC to 5V and GND to GND of Arduino
SDA to A4 and SCL to A5

S20A is 3.3V voltage regulator MIC5205-3.3BM5
*/

#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;
double radToDeg = (180/3.1415);
double thetaX = 0;
double thetaY = 0;
int decimalPrecision = 4;

double gyroX() {
  double gyroX_last_update = micros();
  double deltaTime = (micros() - gyroX_last_update)/1000;
  double gyroXOffset = 0.002483;
  double gyroXRate = (radToDeg*(IMU.getGyroX_rads())) - gyroXOffset;
  thetaX += gyroXRate * deltaTime;
  gyroX_last_update = micros();
  return thetaX;
}

double gyroY() {
  double gyroY_last_update = micros();
  double deltaTime = (micros() - gyroY_last_update)/1000;
  double gyroYOffset = 0.02647;
  double gyroYRate =  (radToDeg*(IMU.getGyroY_rads())) - gyroYOffset;
  thetaY += gyroYRate * deltaTime;
  gyroY_last_update = micros();
  return thetaY;
}


void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
  Serial.print("GyroX: ");
  Serial.print(gyroX(), decimalPrecision); // GyroX
  Serial.print("\t");
  Serial.print("GyroY: ");  
  Serial.print(gyroY(),decimalPrecision); // GyroY
  Serial.print("\n");
}
