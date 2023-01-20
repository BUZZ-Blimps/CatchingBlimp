#pragma once

class CompVEst {

    public:
    CompVEst(double alpha);
    double update(double baro, double accel);
    private:
    double phi = 0.3;
    double alpha;
    double lastPos = 0;
    double lastVEst = 0;
    double lastMills = 0;
};
