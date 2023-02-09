#pragma once
#include "Servo.h"

class ServoWrapper{
    public:
    ServoWrapper(int servoPin);
    void write(double motorValue);
    double getServo();
    double motorSpeed;

    private:
    double lastMotorPos;
    double lastValueTime;
    double motorPosition;
    double minMotorPos;
    double maxMotorPos;
    double targetMotorPos;

    Servo motor;

};