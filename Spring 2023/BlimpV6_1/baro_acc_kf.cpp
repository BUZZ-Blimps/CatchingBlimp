#include "baro_acc_kf.h"

BaroAccKF::BaroAccKF() {
  this->Xkp = {0,
               0,
               0};
               
  this->Pkp = {2,0,0,
               0,2,0,
               0,0,2};
               
  this->Qkp = {0.6,0,0,
               0,0.6,0,
               0,0,0.1};             
}

void BaroAccKF::updateBaroAccel(float baro, float acc, float dt) {
  Matrix<3,3> A = {1, dt, 0.5*dt*dt,
                   0,1,dt,
                   0,0,1};

  Matrix<2,3> H = {1,0,0,
                   0,0,1};

  Matrix<2,2> R = {0.6,0,
                   0,0.001};

  //Prediction Step
  Matrix<3,1> Xk = A*Xkp;
  Matrix<3,3> Pk = A*Pkp*~A+Qkp;

  //update step
  Matrix<2,1> y = {baro, acc};
  Matrix<2,1> V = y-H*Xk;
  Matrix<2,2> S = H*Pk*~H+R;

  Matrix<2,2> S_inv = S;
  bool is_nonsingular = Invert(S_inv);
  if (!is_nonsingular) {
    return;
  }
  
  Matrix<3,2> K = Pk*~H*S_inv;
  Matrix<3,1> Xf = Xk+K*V;
  Matrix<3,3> P = Pk-K*S*~K;

  Xkp = Xf;
  Pkp = P;

  this->x = Xf(0);
  this->v = Xf(1);
  this->a = Xf(2);
}
