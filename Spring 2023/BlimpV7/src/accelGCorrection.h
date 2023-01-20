#pragma once
#include "BasicLinearAlgebra.h"

using namespace BLA;

class AccelGCorrection {
    public:
    AccelGCorrection();
    void updateData(float accX, float accY, float accZ, float pitch, float roll);

    float ax = 0;
    float ay = 0;
    float az = 0;

    float agx = 0;
    float agy = 0;
    float agz = 0;

    private:
    Matrix<3,1> g = {0,0,-9.81};
};