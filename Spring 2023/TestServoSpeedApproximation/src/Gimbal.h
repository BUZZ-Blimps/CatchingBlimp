#pragma once
#include "ServoWrapper.h"
#include "Servo.h"

class Gimbal {
    public:
    Gimbal(int yawPin, int pitchPin, int motorPin,double newDeadband, double newTurnOnCom, double newMinCom, double newMaxCom, double newPhiOffset, double filter);
    bool readyGimbal(bool debug, bool motors_off, double roll, double pitch, double yaw, double up, double forward);
    void updateGimbal(bool ready);
    ServoWrapper pitchServo;

    private:
    double motorCom(double command);

    double deadband;
    double turnOnCom;
    double minCom;
    double maxCom;
    double phiOffset;
    double filter;
    double servoThreshold;
    double nextMotorCom;

    //attach to pin
    ServoWrapper yawServo;
    Servo motor;

    double thetaPos;
    double phiPos1;

    double pi = 3.14159265;

};