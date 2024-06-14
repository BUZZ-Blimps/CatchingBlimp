#pragma once
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

using namespace Eigen;

class GyroEKF {
    public:
    GyroEKF();
    void predict(float dt);
    void updateGyro(float gyrox, float gyroy, float gyroz);
    void updateAccel(float accx, float accy, float accz);

    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    float rollRate = 0;
    float pitchRate = 0;
    float yawRate = 0;
    float rollRateB = 0;
    float pitchRateB = 0;
    float yawRateB = 0;

    private:
    Matrix<float, 9, 1> Xkp;
    Matrix<float, 9, 9> Qkp;
    Matrix<float, 9, 9> Pkp;

    
};