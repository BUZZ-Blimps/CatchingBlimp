#include "tripleBallGrabber.h"
#include "catching_blimp.h"

TripleBallGrabber::TripleBallGrabber(int servoPin, int motorPin) {
  this->servo.attach(servoPin);
  this->motor.attach(motorPin);
  
  // Start closed
  currentAngle = angle_closed;
  targetAngle = currentAngle;
  state = state_closed;
  moveRate = moveRate_auto;

  this->servo.write(currentAngle);
  this->motor.write(1500);
}

void TripleBallGrabber::openGrabber(int blimp_state) {
  updateMoveRate(blimp_state);

  targetAngle = angle_open;
  this->motor.write(1500);
  state = state_open;
}

void TripleBallGrabber::closeGrabber(int blimp_state) {
  updateMoveRate(blimp_state);

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

void TripleBallGrabber::shoot(int blimp_state) {
  updateMoveRate(blimp_state);

  targetAngle = angle_open;
  currentAngle = targetAngle;
  state = state_shooting;
  this->motor.write(1850);
}

void TripleBallGrabber::updateMoveRate(int blimp_state){
  if(blimp_state == blimpState::manual){
    moveRate = moveRate_manual;
  }else{
    moveRate = moveRate_auto;
  }
}