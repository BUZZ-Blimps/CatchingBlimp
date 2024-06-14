#pragma once

#include <vector>
#include "Arduino.h"
#include "String.h"

class SerialData {
    public:
    std::vector<std::vector<double> > getTarget();
    std::vector<double> getManualComs();
    int getState();
    void update(std::vector<double> newMData);
    bool hasBeenUpdated();
    std::vector<std::vector<double> > target;
    
    private:
    std::vector<double> mData;
    double state = 0;
    bool hasUpdated;

};
