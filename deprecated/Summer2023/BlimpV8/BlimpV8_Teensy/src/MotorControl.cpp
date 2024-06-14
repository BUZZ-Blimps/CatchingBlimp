#include "MotorControl.h"

void MotorControl::update(double forward, double translation, double up, double yaw, double roll) {
  this->forwardLeft = -(forward - yaw);
  this->forwardRight = -(forward + yaw);
  this->upLeft = -(up + roll);
  this->upRight = (up - roll);
  this->yawLeft = translation;

  this->yawRight = translation;
}