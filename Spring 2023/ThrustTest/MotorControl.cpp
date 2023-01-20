#include "MotorControl.h"
#include "Arduino.h"

MotorControl::MotorControl(int motorPin, double newdeadband, double newturnOnCom, double newminCom, double newmaxCom) {
        this->deadband = newdeadband;
        this->turnOnCom = newturnOnCom;
        this->minCom = newminCom;
        this->maxCom = newmaxCom;
        motor.attach(motorPin);
        motor.write(1500);
}

double MotorControl::motorCom(double command) {
    //input from -1000, to 1000 is expected from controllers
    double adjustedCom = 1500;
    
    if (abs(command) <= deadband/2) {
        adjustedCom = 1500;
    } else if (command > deadband/2) {
        double xo1 = deadband/2;
        double yo1 = turnOnCom+1500;
        double m1 = (maxCom-yo1)/(1000-xo1);
        adjustedCom = m1*command-m1*xo1+yo1;
    } else if (command < deadband/2) {
        double xo2 = -deadband/2;
        double yo2 = -turnOnCom+1500;
        double m2 = (yo2-minCom)/(xo2-(-1000));
        adjustedCom = m2*command-m2*xo2+yo2;
    } else {
        //should never happend, but write 1500 anyway for safety
        adjustedCom = 1500;
    }
    
    this->motor.write(adjustedCom);
    return adjustedCom;
}
