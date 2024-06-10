#include "tripleBallGrabber.h"
#include <math.h>
// #include "catching_blimp.h"

TripleBallGrabber::TripleBallGrabber() {
  // Start closed
  currentAngle = angle_closed;
  targetAngle = currentAngle;
  state = state_closed;
  moveRate = moveRate_slow;
}

void TripleBallGrabber::ballgrabber_init(int servoPin, int motorPin){
  this->Servo.servo_PIN(servoPin);
  this->motor.brushless_PIN(motorPin);
  this->Servo.servo_angle(currentAngle);
  this->motor.brushless_thrust(0);
}

void TripleBallGrabber::openGrabber(int blimp_state) {
  updateMoveRate(blimp_state);

  targetAngle = angle_open;
  this->motor.brushless_thrust(0);
  state = state_open;
}

void TripleBallGrabber::closeGrabber(int blimp_state) {
  //updateMoveRate(blimp_state);
  moveRate = moveRate_fast; // Close fast, regardless of state

  targetAngle = angle_closed;
  this->motor.brushless_thrust(0);
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
  this->Servo.servo_angle(round(currentAngle));
}

void TripleBallGrabber::shoot(int blimp_state) {
  updateMoveRate(blimp_state);

  targetAngle = angle_open;
  currentAngle = targetAngle;
  state = state_shooting;
  this->motor.brushless_thrust(100);
}

void TripleBallGrabber::updateMoveRate(int blimp_state){
  if(blimp_state == 0){ // blimpState::manual
    moveRate = moveRate_fast;
  }else{
    moveRate = moveRate_slow;
  }
}