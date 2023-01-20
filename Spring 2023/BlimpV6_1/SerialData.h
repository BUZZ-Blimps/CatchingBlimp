#pragma once

#include <vector>
#include "Arduino.h"
#include "String.h"

class SerialData {
    public:
    std::vector<std::vector<double> > getBalloonData();
    std::vector<std::vector<double> > getDGoalData();
    std::vector<std::vector<double> > getAGoalData();
    std::vector<std::vector<double> > getEBlimpData();
    std::vector<std::vector<double> > getFBlimpData();
    std::vector<double> getManualComs();
    int getState();
    void update(std::vector<double> newMData, std::vector<std::vector<std::vector<double>>> newAData);
    bool hasBeenUpdated();
    
    private: 
    std::vector<std::vector<std::vector<double> > > aData;
    std::vector<double> mData;
    double state = 0;
    bool hasUpdated;

};
