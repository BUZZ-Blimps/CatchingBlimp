#include "SerialData.h"

std::vector<double> SerialData::getManualComs() {
    return this->mData;
}

int SerialData::getState() {
    return this->state;
}

void SerialData::update(std::vector<double> newMData) {
    this->mData.clear();
    this->mData = newMData;
    this->hasUpdated = true;
}

bool SerialData::hasBeenUpdated() {
  bool tempValue = this->hasUpdated;
  this->hasUpdated = false;
  return tempValue;
}
