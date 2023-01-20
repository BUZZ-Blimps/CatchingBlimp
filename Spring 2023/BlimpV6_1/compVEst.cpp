#include "compVEst.h"
#include "Arduino.h"

CompVEst::CompVEst(double newAlpha) {
    this->alpha = newAlpha;
    this->lastMills = millis();
}

double CompVEst::update(double baro, double accel) {
    double est = phi*((baro-this->lastPos)/((micros()-this->lastMills)/1000000.0))+(1.0-phi)*(this->lastVEst+accel*((micros()-this->lastMills)/1000000.0));
    this->lastPos = baro;
    this->lastMills = micros();
    this->lastVEst = est;

    if (phi > alpha) {
      phi = phi/1.05;
    } else {
      phi = alpha;
    }

    return est;
}
