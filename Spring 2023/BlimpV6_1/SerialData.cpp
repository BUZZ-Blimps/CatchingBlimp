#include "SerialData.h"

std::vector<std::vector<double> > SerialData::getBalloonData() {
    std::vector<std::vector<double>> object;
    if (aData.size() == 5) {
      return this->aData[0];
    } else {
      return object;
    }
}

std::vector<std::vector<double> > SerialData::getDGoalData() {
    std::vector<std::vector<double>> object;
    if (aData.size() == 5) {
      return this->aData[1];
    } else {
      return object;
    }
}

std::vector<std::vector<double> > SerialData::getAGoalData() {
    std::vector<std::vector<double>> object;
    if (aData.size() == 5) {
      return this->aData[2];
    } else {
      return object;
    }
}

std::vector<std::vector<double> > SerialData::getEBlimpData() {
    std::vector<std::vector<double>> object;
    if (aData.size() == 5) {
      return this->aData[3];
    } else {
      return object;
    }
}

std::vector<std::vector<double> > SerialData::getFBlimpData() {
    std::vector<std::vector<double>> object;
    if (aData.size() == 5) {
      return this->aData[4];
    } else {
      return object;
    }
}

std::vector<double> SerialData::getManualComs() {
    return this->mData;
}

int SerialData::getState() {
    return this->state;
}

void SerialData::update(std::vector<double> newMData, std::vector<std::vector<std::vector<double>>> newAData) {
    this->mData.clear();
    this->aData.clear();
    this->mData = newMData;
    this->aData = newAData;
    this->hasUpdated = true;
}

bool SerialData::hasBeenUpdated() {
  bool tempValue = this->hasUpdated;
  this->hasUpdated = false;
  return tempValue;
}
