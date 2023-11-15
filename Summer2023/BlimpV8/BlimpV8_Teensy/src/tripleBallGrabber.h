#pragma once

#include "Arduino.h"
#include "Servo.h"

class TripleBallGrabber {  
  public:
    TripleBallGrabber(int servoPin, int motorPin);
    void openGrabber();
    void closeGrabber();
    void update();
    void shoot();

    int state = 0;
    double currentAngle = 0; // [deg]
    double targetAngle = 0; // [deg]

  private:
    Servo servo;
    Servo motor;
    double moveRate = 30.0; // [deg/s]
    double lastCommandTime = 0; // [s]

    const double angle_closed = 83; // [deg]
    const double angle_open = 20; // [deg]

    const int state_closed = 0;
    const int state_open = 1;
    const int state_shooting = 2;
};
