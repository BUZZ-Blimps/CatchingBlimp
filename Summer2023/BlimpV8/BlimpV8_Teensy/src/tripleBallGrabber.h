#pragma once

#include "Arduino.h"
#include "Servo.h"

class TripleBallGrabber {  
  public:
    TripleBallGrabber();
    void ballgrabber_init(int servoPin, int motorPin);
    void openGrabber(int blimp_state);
    void closeGrabber(int blimp_state);
    void update();
    void shoot(int blimp_state);
    void updateMoveRate(int blimp_state);

    int state = 0;
    double currentAngle = 0; // [deg]
    double targetAngle = 0; // [deg]

  private:
    Servo servo;
    Servo motor;
    double moveRate;
    double lastCommandTime = 0; // [s]

    const double moveRate_fast = 180.0; // [deg/s]
    const double moveRate_slow = 45.0; // [deg/s]

    const double angle_closed = 83; // [deg]
    const double angle_open = 20; // [deg]

    const int state_closed = 0;
    const int state_open = 1;
    const int state_shooting = 2;
};
