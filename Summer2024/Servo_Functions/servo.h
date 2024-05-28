#pragma once
#include "wiringPi.h"

class servo{
    public:
    void servo_setup(int PIN);
    double servo_angle(double angle);
    void servo_PIN(int PIN);
    double get_angle();


    private:
    double curr_angle;
    int pin;
    unsigned int ccr;		// Capture/Compare Register (Duty Cycle)
	unsigned int arr;		// Auto-Reload Register (Period)
	unsigned int div;
	unsigned int div_stepping;
};