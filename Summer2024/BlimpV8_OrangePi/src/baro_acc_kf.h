#pragma once
#include "BasicLinearAlgebra.h"

using namespace BLA;

class BaroAccKF {
  public:
  BaroAccKF();
  void predict(float dt);
  void updateBaro(float baro);
  void updateAccel(float acc);
  float x;
  float v;
  float a;
  float b;

  private:
  Matrix<4,1> Xkp;
  Matrix<4,4> Pkp;
  Matrix<4,4> Qkp;

  float lastAccelTime = 0.0;
  
};
