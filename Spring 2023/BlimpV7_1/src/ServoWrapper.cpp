#include "ServoWrapper.h"
#include <Arduino.h>

void ServoWrapper::attach(int servoPin){
    lastMotorPos=0; // degrees from start
    lastValueTime=0;
    motorSpeed=675; // (degrees/s) manually estimated
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
    //Serial.print("New motor value: ");
    //Serial.println(String(motorValue));
}

void ServoWrapper::updateApproximation(){
    double currentTime=micros()/1000000.0;
    double deltaTime=currentTime-lastValueTime; // Time since last time check
    double errorPos=targetMotorPos-lastMotorPos;
    if(errorPos>0){
        currentMotorPos=lastMotorPos+deltaTime*motorSpeed; // Predictive model positive direction
        if(currentMotorPos>targetMotorPos) currentMotorPos=targetMotorPos;
        /*
        Serial.print("Error>0: LastPos=");
        Serial.print(lastMotorPos);
        Serial.print(" CurrentMotorPos=");
        Serial.print(currentMotorPos);
        Serial.print(" deltaTime=");
        Serial.print(deltaTime);
        Serial.print(" deltaPos=");
        Serial.println(deltaTime*motorSpeed);
        */
    }else if(errorPos<0){
        currentMotorPos=lastMotorPos-deltaTime*motorSpeed; // Predictive model negative direction
        if(currentMotorPos<targetMotorPos) currentMotorPos=targetMotorPos;
        /*
        Serial.print("Error<0: LastPos=");
        Serial.print(lastMotorPos);
        Serial.print(" CurrentMotorPos=");
        Serial.print(currentMotorPos);
        Serial.print(" deltaTime=");
        Serial.print(deltaTime);
        Serial.print(" deltaPos=");
        Serial.println(-deltaTime*motorSpeed);
        */
    }else{
        currentMotorPos=lastMotorPos;
    }
    lastValueTime=currentTime;
    lastMotorPos=currentMotorPos;
}
double ServoWrapper::getServo(){
    updateApproximation();
    return currentMotorPos;
}