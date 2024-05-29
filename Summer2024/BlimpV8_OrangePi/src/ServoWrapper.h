#pragma once
#include "servo.h"

class ServoWrapper{
    public:
    void attach(int servoPin);
    void write(double motorValue);
    double getServo();
    void updateApproximation();
    double motorSpeed;

    private:
    double lastMotorPos;
    double lastValueTime;
    double motorPosition;
    double minMotorPos;
    double maxMotorPos;
    double targetMotorPos;
    double currentMotorPos;

    servo motor;

};