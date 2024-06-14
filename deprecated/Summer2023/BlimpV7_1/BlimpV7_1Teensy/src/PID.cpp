#include <Arduino.h>
#include <cmath>
#include "PID.h"

using namespace std;

/**
 * Implementation
 */
PID::PID(double kp, double ki, double kd) :
    _kp(kp),
    _ki(ki),
    _kd(kd),
    _error(0),
    _pre_error(0),
    _integral(0),
    _i_limit(0),
    _d_limit(0),
    _limit_output(false)
{
}

PID::~PID(){
}

void PID::setOutputLimits(double min, double max) {
    _out_min = min;
    _out_max = max;
    _limit_output = true;
}

void PID::setILimit(double iLimit) {
    _i_limit = abs(iLimit);
}

void PID::setDLimit(double dLimit) {
    _d_limit = abs(dLimit);
}

double PID::calculate(double setpoint, double pv, double dt) {
    // Calculate error
    _error = setpoint - pv;

    // Proportional term
    double p_out = _kp * _error;

    // Integral term
    _integral += _error * dt;
    double i_out = _ki * _integral;

    //Integral windup limit
    if (_i_limit > 0) {
        i_out = constrain(i_out, -1 * _i_limit, _i_limit);
    }

    // Derivative term (zero if dt == 0)
    double d_out = 0;
    if (dt != 0) {
        double derivative = (_error - _pre_error) / dt;
        d_out = _kd * derivative;

        if (_d_limit > 0) {
            d_out = constrain(d_out, -1 * _d_limit, _d_limit);
        }
    }

    // Calculate total output
    double output = p_out + i_out + d_out;

    // Restrict to max/min
    if (_limit_output) {
        output = constrain(output, _out_min, _out_max);
    }

    // Save error to previous error
    _pre_error = _error;

    return output;
}

void PID::reset() {
    _error = 0;
    _pre_error = 0;
    _integral = 0;
}