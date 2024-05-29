#pragma once
#include "ServoWrapper.h"

class Gimbal {
    public:
    // Gimbal();
    void gimbal_init(int yawPin, int pitchPin, int motorPin,double newDeadband, double newTurnOnCom, double newMinCom, double newMaxCom, double newPhiOffset, double filter);
    bool readyGimbal(bool debug, bool motors_off, double roll, double pitch, double yaw, double up, double forward);
    void updateGimbal(bool ready);
    double servoThreshold;

    private:
    double motorCom(double command);

    double deadband;
    double turnOnCom;
    double minCom;
    double maxCom;
    double phiOffset;
    double filter;
    double nextMotorCom;


    //attach to pin
    ServoWrapper yawServo;
    ServoWrapper pitchServo;
    ServoWrapper motor;

    double thetaPos;
    double phiPos1;

    double pi = 3.1415927;

};