#include "brushless.h"


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
    this->curr_thrust = (thrust*1.2) + 300;
    pwmWrite(this->pin, this->curr_thrust);
    return this->curr_thrust;
}

double brushless::get_thrust(){
    return((this->curr_thrust - 300)*2);
}