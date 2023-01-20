#pragma once
#include "Servo.h"

class MotorControl {
    public:
    MotorControl(int motorPin, double deadband, double turnOnCom, double minCom, double maxCom);
    double motorCom(double command);

    private:
    Servo motor;
    double deadband;
    double turnOnCom;
    double minCom;
    double maxCom;
};
