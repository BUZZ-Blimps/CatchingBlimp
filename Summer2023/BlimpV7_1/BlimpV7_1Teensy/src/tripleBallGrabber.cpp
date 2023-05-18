#include "tripleBallGrabber.h"

TripleBallGrabber::TripleBallGrabber(int servoPin, int motorPin) {
  Serial.println("Initializing Ball Grabber");
  this->servo.attach(servoPin);
  this->motor.attach(motorPin);
  
  this->servo.write(83);
  this->motor.write(1500);
  Serial.println("Setup Comlete");
}

void TripleBallGrabber::openGrabber() {
  this->servo.write(15);  //15 or 151
  this->motor.write(1500);
  this->state = 1;
}

void TripleBallGrabber::closeGrabber() {
  this->servo.write(83);
  this->motor.write(1500);
  this->state = 0;
}

void TripleBallGrabber::shoot() {
  this->servo.write(15);  //15 or 151
  this->motor.write(1850);
  this->state = 2;
}
