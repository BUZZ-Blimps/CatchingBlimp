#include "brushless.h"
#include <cstdio>


void brushless::brushless_setup(int PIN){
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

void brushless::brushless_PIN(int PIN){
    this->pin = PIN;
}

double brushless::brushless_thrust(double thrust){
    if (1000<= thrust <= 2000){
        this->curr_thrust = (thrust*0.1) + 150;
        pwmWrite(this->pin, this->curr_thrust);
        return this->curr_thrust;
    }
    else {
        printf("Thrust out of range!");
        return(this->curr_thrust);
    }
}

double brushless::get_thrust(){
    return((this->curr_thrust - 150)*0.1);
}