#pragma once

#include "Arduino.h"
#include "Servo.h"
#include "EMAFilter.h"

class MotorControl {
    public:
    MotorControl(int LSPin, int RSPin, int LMPin, int RMPin, double newdeadband, double newturnOnCom, double newminCom, double newmaxCom, double servoFilter);
    void update(double pitch, double forward, double up, double yaw);
    void writeLServo(double angle);
    void writeRServo(double angle);

    EMAFilter servoRFilter;
    EMAFilter servoLFilter;

    private:
    Servo LServo;
    Servo RServo;
    Servo LMotor;
    Servo RMotor;
    double deadband;
    double turnOnCom;
    double minCom;
    double maxCom;
    
    double motorCom(double command);
};
