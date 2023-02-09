#include "ServoWrapper.h"
#include <Arduino.h>

void ServoWrapper::attach(int servoPin){
    lastMotorPos=0; // degrees from start
    lastValueTime=0;
    motorSpeed=200; // (degrees/s) Taken from data sheet -- 60 degrees in 0.3 seconds
    minMotorPos=0;
    maxMotorPos=180; // Subject to change with the addition of the 270 degree servos
    targetMotorPos=90; // arbitrary startup target

    motor.attach(servoPin);
    write(targetMotorPos); // arbitrary initial position
}

void ServoWrapper::write(double motorValue){
    updateApproximation();
    motor.write(motorValue);
    targetMotorPos=motorValue;
}

void ServoWrapper::updateApproximation(){
    double currentTime=micros()/1000;
    double deltaTime=currentTime-lastValueTime; // Time since last time check
    double errorPos=targetMotorPos-lastMotorPos;
    double currentPos;
    if(errorPos>0){
        currentPos=lastMotorPos+deltaTime*motorSpeed; // Predictive model positive direction
        if(currentPos>targetMotorPos) currentPos=targetMotorPos;
    }else if(errorPos<0){
        currentPos=lastMotorPos-deltaTime*motorSpeed; // Predictive model negative direction
        if(currentPos<targetMotorPos) currentPos=targetMotorPos;
    }else{
        currentPos=lastMotorPos;
    }
    lastValueTime=currentTime;
    lastMotorPos=currentPos;
}
double ServoWrapper::getServo(){
    updateApproximation();
    return currentPos;
}