#include "EMAFilter.h"
#include "Arduino.h"

EMAFilter::EMAFilter(double newAlpha) {
    this->alpha = newAlpha;
    this->last = 0;
}

EMAFilter::EMAFilter() {
  this->alpha = 1;
  this->last = 0;
}

void EMAFilter::setAlpha(double newAlpha) {
  this->alpha = newAlpha;
}

void EMAFilter::setInitial(double initial) {
  this->last = initial;
}

double EMAFilter::filter(double current) {

    if (isnan(current)) {
      return this->last;
    }
    
    double next = (this->alpha) * current + ((double)1.0 - (this->alpha)) * (this->last);
    this->last = next;
    return next;
}
