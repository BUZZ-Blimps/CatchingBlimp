#pragma once
#include <eigen3/Eigen/Dense>

using namespace Eigen;

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
    Eigen::Vector3f g;
};