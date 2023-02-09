#include <Arduino.h>
#include "Gimbal.h"
#include "ServoWrapper.h"
#include <cmath>

Gimbal rightGimbal(10, 9, 5, 0, 30, 1000, 2000, 135, 1);
//ServoWrapper servo;
double timeDelay = 3; // seconds
double printFrequency = 5; // Hz
double testMag = 300;
bool motors_off = false;
bool debug = false;

double timeStart;
double timeSwap = 3;

void loop_testGimbalLong();
void loop_testGimbalShort();
void loop_testGimbalQuick();

void setup() {
  //servo.attach(9);
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello!");

  timeStart = micros()/1000000.0;
}

void loop() {
  loop_testGimbalQuick();
}

void loop_testGimbalLong(){
  // Point up
  Serial.println("About to try pointing up...");
  delay(1000);
  Serial.println("Trying to point up!");
  //servo.write(0);
  rightGimbal.readyGimbal(true, motors_off, 0, 0, 0, testMag, 0);
  rightGimbal.updateGimbal(!motors_off);
  for(int i=0; i<(timeDelay*printFrequency); i++){
    double servoPos = rightGimbal.pitchServo.getServo();
    //double servoPos = servo.getServo();
    Serial.print("Servo Position: ");
    Serial.println(String(servoPos));
    delay(1000.0/printFrequency);
  }

  // Point forward
  Serial.println("About to try pointing forward...");
  delay(1000);
  Serial.println("Trying to point forward!");
  //servo.write(0);
  rightGimbal.readyGimbal(true, motors_off, 0, 0, 0, 0, testMag);
  rightGimbal.updateGimbal(!motors_off);
  for(int i=0; i<(timeDelay*printFrequency); i++){
    double servoPos = rightGimbal.pitchServo.getServo();
    //double servoPos = servo.getServo();
    Serial.print("Servo Position: ");
    Serial.println(String(servoPos));
    delay(1000.0/printFrequency);
  }

  // Point down
  Serial.println("About to try pointing down...");
  delay(1000);
  Serial.println("Trying to point down!");
  rightGimbal.readyGimbal(true, motors_off, 0, 0, 0, -testMag, 0);
  rightGimbal.updateGimbal(!motors_off);
  //servo.write(180);
  for(int i=0; i<(timeDelay*printFrequency); i++){
    double servoPos = rightGimbal.pitchServo.getServo();
    //double servoPos = servo.getServo();
    Serial.print("Servo Position: ");
    Serial.println(String(servoPos));
    delay(1000.0/printFrequency);
  }

  // Point forward
  Serial.println("About to try pointing forward...");
  delay(1000);
  Serial.println("Trying to point forward!");
  //servo.write(0);
  rightGimbal.readyGimbal(true, motors_off, 0, 0, 0, 0, testMag);
  rightGimbal.updateGimbal(!motors_off);
  for(int i=0; i<(timeDelay*printFrequency); i++){
    double servoPos = rightGimbal.pitchServo.getServo();
    //double servoPos = servo.getServo();
    Serial.print("Servo Position: ");
    Serial.println(String(servoPos));
    delay(1000.0/printFrequency);
  }
}

void loop_testGimbalShort(){
  // Point up
  Serial.println("About to try pointing up...");
  delay(1000);
  Serial.println("Trying to point up!");
  //servo.write(0);
  rightGimbal.readyGimbal(true, motors_off, 0, 0, 0, testMag, 0);
  rightGimbal.updateGimbal(!motors_off);
  delay(timeDelay*1000);

  // Point forward
  Serial.println("About to try pointing forward...");
  delay(1000);
  Serial.println("Trying to point forward!");
  //servo.write(0);
  rightGimbal.readyGimbal(true, motors_off, 0, 0, 0, 0, testMag);
  rightGimbal.updateGimbal(!motors_off);
  delay(timeDelay*1000);

  // Point down
  Serial.println("About to try pointing down...");
  delay(1000);
  Serial.println("Trying to point down!");
  rightGimbal.readyGimbal(true, motors_off, 0, 0, 0, -testMag, 0);
  rightGimbal.updateGimbal(!motors_off);
  //servo.write(180);
  delay(timeDelay*1000);

  // Point forward
  Serial.println("About to try pointing forward...");
  delay(1000);
  Serial.println("Trying to point forward!");
  //servo.write(0);
  rightGimbal.readyGimbal(true, motors_off, 0, 0, 0, 0, testMag);
  rightGimbal.updateGimbal(!motors_off);
  delay(timeDelay*1000);
}

void loop_testGimbalQuick(){
  double timeCurr = micros()/1000000.0;
  double elapsedTime = timeCurr - timeStart;
  double modElapsedTime = elapsedTime - (2*timeSwap)*((int)(elapsedTime/(2*timeSwap)));
  //Serial.print("ModElapsedTime: ");
  //Serial.println(modElapsedTime);

  bool ready;
  /*
  if(modElapsedTime < (1*timeSwap)){
    //Up
    ready = rightGimbal.readyGimbal(debug, motors_off, 0, 0, 0, testMag, 0);
  }else if(modElapsedTime < (2*timeSwap)){
    //Foward
    ready = rightGimbal.readyGimbal(debug, motors_off, 0, 0, 0, 0, testMag);
  }else if(modElapsedTime < (3*timeSwap)){
    //Down
    ready = rightGimbal.readyGimbal(debug, motors_off, 0, 0, 0, -testMag, 0);
  }else{
    //Backward
    ready = rightGimbal.readyGimbal(debug, motors_off, 0, 0, 0, 0, -testMag);
  }
  */

  if(modElapsedTime < (1*timeSwap)){
    //Up
    ready = rightGimbal.readyGimbal(debug, motors_off, 0, 0, 0, testMag, 0.6*testMag);
  }else{
    //Down
    ready = rightGimbal.readyGimbal(debug, motors_off, 0, 0, 0, -testMag, 0.6*testMag);
  }

  rightGimbal.updateGimbal(ready);
}

