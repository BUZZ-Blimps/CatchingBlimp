#include "servo.h"
#include <cstdio>


void servo::servo_setup(int PIN){
    this->ccr = 500;
	this->arr = 1000;
	this->div = 120;
	this->div_stepping = 2;
    wiringPiSetup();
    this->pin = PIN;
	pinMode(PIN, PWM_OUTPUT);
    pwmSetRange(PIN, this->arr);
	pwmSetClock(PIN, this->div);
	pwmWrite(PIN, this->ccr);
}

void servo::servo_PIN(int PIN){
    this->pin = PIN;
}

double servo::servo_angle(double angle){
    if (0 <= angle && angle <= 180){
        this->curr_angle = (angle*(1.5)) + 160;
        pwmWrite(this->pin, this->curr_angle);
        return this->curr_angle;
    }
    else {
        printf("Servo angle out of range!\n");
        return(this->curr_angle);
    }
}

double servo::get_angle(){
    return((this->curr_angle - 160)/1.5);
}