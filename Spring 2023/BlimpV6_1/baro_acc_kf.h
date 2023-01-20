#pragma once
#include "BasicLinearAlgebra.h"

using namespace BLA;

class BaroAccKF {
  public:
  BaroAccKF();
  void updateBaroAccel(float baro, float acc, float dt);
  float x;
  float v;
  float a;
  float c = 0.1;
  int count = 0;
  int predict = 1;

  private:
  Matrix<3,1> Xkp;
  Matrix<3,3> Pkp;
  Matrix<3,3> Qkp;
};
