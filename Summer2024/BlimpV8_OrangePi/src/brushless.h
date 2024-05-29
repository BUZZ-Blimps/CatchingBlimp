#pragma once
#include <wiringPi.h>

class brushless{
    public:
    void brushless_setup(int PIN);
    double brushless_thrust(double thrust);
    void brushless_PIN(int PIN);
    double get_thrust();


    private:
    double curr_thrust;
    int pin;
    unsigned int ccr;		// Capture/Compare Register (Duty Cycle)
	unsigned int arr;		// Auto-Reload Register (Period)
	unsigned int div;
	unsigned int div_stepping;
};