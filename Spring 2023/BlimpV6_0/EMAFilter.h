#pragma once

class EMAFilter {
    private:
    double last;
    double alpha;
    
    public:
    EMAFilter(double alpha);
    double filter(double current);
};
