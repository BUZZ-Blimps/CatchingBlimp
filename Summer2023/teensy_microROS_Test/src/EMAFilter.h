#pragma once

class EMAFilter {
    private:
    double alpha;
    
    public:
    EMAFilter(double alpha);
    EMAFilter();
    void setAlpha(double alpha);
    void setInitial(double initial);
    double filter(double current);
    double last;
};
