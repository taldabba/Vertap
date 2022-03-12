/*
    Kalman Filter Example for MPU6050. Output for processing.
    Read more: http://www.jarzebski.pl/arduino/rozwiazania-i-algorytmy/odczyty-pitch-roll-oraz-filtr-kalmana.html
    GIT: https://github.com/jarzebski/Arduino-KalmanFilter
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/


#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>

MPU6050 mpu;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

float savedPitch = 0;
float savedRoll = 0;

float errorPitch = 0;
float errorRoll = 0;
float errorSum = 0;


int redPin = 8;
int greenPin = 9;
int bluePin = 10;

int buttonPin = 7;
int buttonState = 0;

void setup() 
{

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buttonPin, INPUT);
  
  Serial.begin(115200);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }
 
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
}

void RGB_color(float redValue, float greenValue, float blueValue)
 {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}



void loop()
{
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;

  // Kalman filter
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);

//  Serial.print(kalPitch);
//  Serial.print(":");
//  Serial.print(kalRoll);
//  Serial.print(":");
//  Serial.print(savedPitch);
//  Serial.print(":");
//  Serial.print(savedRoll);
//  Serial.print(":");
//  Serial.println(errorSum);
//  Serial.println();
  
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    savedPitch = kalPitch;
    savedRoll = kalRoll;
  }

  float slope = (255/75);
  errorPitch = slope * abs(kalPitch - savedPitch);
  errorRoll = slope * abs(kalRoll - savedRoll);
  errorSum = errorPitch + errorRoll;
  
  if (errorSum >= 120) {
    RGB_color((255.0), (0.0), 0.0);
  }
  else {
    RGB_color((0.0 + 3*errorSum), (255.0 - (2*errorSum)), 0.0);
  }
}
