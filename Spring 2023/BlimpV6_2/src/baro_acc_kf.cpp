#include "baro_acc_kf.h"

BaroAccKF::BaroAccKF() {
  this->Xkp = {0,
               0,
               0,
               0};
               
  this->Pkp = {2,0,0,0,
               0,2,0,0,
               0,0,2,0,
               0,0,0,2};
               
  this->Qkp = {0.001,0,0,0,
               0,0.01,0,0,
               0,0,0.1,0,
               0,0,0,0}; 
}

void BaroAccKF::predict(float dt) {
  Matrix<4,4> A = {1,dt,0,0,
                   0,1,dt,-dt,
                   0,0,1,0,
                   0,0,0,1};

  //Prediction Step
  Matrix<4,1> Xk = A*Xkp;
  Matrix<4,4> Pk = A*Pkp*~A+Qkp;

  Xkp = Xk;
  Pkp = Pk;

  this->x = Xkp(0);
  this->v = Xkp(1);
  this->a = Xkp(2);
  this->b = Xkp(3);
}

void BaroAccKF::updateBaro(float baro) {
  Matrix<1,4> H = {1,0,0,0};
  Matrix<1,1> R = {0.36};   

  //update step
  Matrix<1,1> y = {baro};
  Matrix<1,1> V = y-H*Xkp;
  Matrix<1,1> S = H*Pkp*~H+R;

  Matrix<1,1> S_inv = S;
  bool is_nonsingular = Invert(S_inv);
  if (!is_nonsingular) {
    return;
  }
  
  Matrix<4,1> K = Pkp*~H*S_inv;
  Xkp = Xkp+K*V;
  Pkp = Pkp-K*S*~K;

  this->x = Xkp(0);
  this->v = Xkp(1);
  this->a = Xkp(2);
  this->b = Xkp(3);
}

void BaroAccKF::updateAccel(float acc) {

  Matrix<1,4> H = {0, 0, 1, 0};
  Matrix<1,1> R = {0.001};

  //update step
  Matrix<1,1> y = {acc};
  Matrix<1,1> V = y-H*Xkp;
  Matrix<1,1> S = H*Pkp*~H+R;

  Matrix<1,1> S_inv = S;
  bool is_nonsingular = Invert(S_inv);
  if (!is_nonsingular) {
    return;
  }
  
  Matrix<4,1> K = Pkp*~H*S_inv;
  Xkp = Xkp+K*V;
  Pkp = Pkp-K*S*~K;

  this->x = Xkp(0);
  this->v = Xkp(1);
  this->a = Xkp(2);
  this->b = Xkp(3);
}
