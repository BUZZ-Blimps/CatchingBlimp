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

void EMAFilter::reset(){
  this->initialized = false;
}

void EMAFilter::setAlpha(double newAlpha) {
  this->alpha = newAlpha;
}

void EMAFilter::setInitial(double initial) {
  this->last = initial;
}

double EMAFilter::filter(double current) {
  // Check for nan, return nan lmao (don't mess up state of filter)
  if (isnan(current)) {
    return current;
  }

  double next;
  // Check if filter is initialized
  if(!this->initialized){
    // Re-initialize filter with current value
    next = current;
    this->initialized = true;
  }else{
    // Already initialized...
    next = (this->alpha) * current + ((double)1.0 - (this->alpha)) * (this->last);
  }

  this->last = next;
  return next;
}
