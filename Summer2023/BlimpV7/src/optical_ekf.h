#pragma once
#include "BasicLinearAlgebra.h"

using namespace BLA;

class OpticalEKF {
  public:
  OpticalEKF(float a_new, float b1_new, float b2_new);
  void predict(float dt);
  void updateBaro(float baro);
  void updateOptical(float optical);
  void updateGyroX(float gyrox);
  void updateGyroZ(float gyroy);
  void updateAccelx(float accx);
  
  float z = 0;
  float opt = 0;
  float gyrox = 0;
  float gyroz = 0;
  float v = 0;
  float ax = 0;
  float b = 0;

  private:
  Matrix<7,1> Xkp;
  Matrix<7,7> Pkp;
  Matrix<7,7> Qkp;

  float a = 0.001;
  float b1 = 0;
  float b2 = 0;

  float baroR = 0.36;
  float opticalR = 0.01;
  float gyroXR = 0.1;
  float gyroZR = 0.1;
  float accelXR = 0.001;
};