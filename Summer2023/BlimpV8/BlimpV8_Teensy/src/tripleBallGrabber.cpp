#include "tripleBallGrabber.h"

TripleBallGrabber::TripleBallGrabber(int servoPin, int motorPin) {
  // Serial.println("Initializing Ball Grabber");
  this->servo.attach(servoPin);
  this->motor.attach(motorPin);
  
  // Start closed
  currentAngle = angle_closed;
  targetAngle = currentAngle;
  state = state_closed;

  this->servo.write(currentAngle);
  this->motor.write(1500);
  // Serial.println("Setup Comlete");
}

// int pose = 0;

//68 degree sweep

void TripleBallGrabber::openGrabber() {
  // for (pose = 83; pose >15; pose -=1){
  //   this->servo.write(pose);
  //   delay(15);
  // }
  // // this->servo.write(15);  //15 or 151
  // this->motor.write(1500);
  // this->state = 1;

  targetAngle = angle_open;
  this->motor.write(1500);
  state = state_open;
}

void TripleBallGrabber::closeGrabber() {
  // for (pose = 15; pose <83; pose +=1){
  //   this->servo.write(pose);
  //   delay(15);
  // }
  // this->motor.write(1500);
  // this->state = 0;

  targetAngle = angle_closed;
  this->motor.write(1500);
  state = state_closed;
}

void TripleBallGrabber::update(){
  double currentTime = micros()/1000000.0;
  double elapsedTime = currentTime - lastCommandTime;
  lastCommandTime = currentTime;

  double maxAngleMovement = elapsedTime * moveRate;

  double errorAngle = targetAngle - currentAngle;

  double deltaAngle;
  if(errorAngle > maxAngleMovement){
    deltaAngle = maxAngleMovement;
  }else if(errorAngle < -maxAngleMovement){
    deltaAngle = -maxAngleMovement;
  }else{
    // Error is extremely close
    deltaAngle = errorAngle;
  }

  currentAngle += deltaAngle;
  this->servo.write(round(currentAngle));
}

void TripleBallGrabber::shoot() {
  targetAngle = angle_open;
  state = state_shooting;
  this->motor.write(1850);
}
