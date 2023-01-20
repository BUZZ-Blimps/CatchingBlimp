#pragma once

#include "Arduino.h"
#include "Servo.h"

class TripleBallGrabber {  
  public:
  TripleBallGrabber(int servoPin, int motorPin);
  void openGrabber();
  void closeGrabber();
  void shoot();

  int state = 0;

  private:
  Servo servo;
  Servo motor;
};
