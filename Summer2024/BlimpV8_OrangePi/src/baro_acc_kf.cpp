#include "baro_acc_kf.h"

BaroAccKF::BaroAccKF() {
  this->Xkp  <<0,
               0,
               0,
               0;
               
  this->Pkp   <<2,0,0,0,
               0,2,0,0,
               0,0,2,0,
               0,0,0,2;
               
  this->Qkp  <<0.001,0,0,0,
               0,0.01,0,0,
               0,0,0.1,0,
               0,0,0,0; 
}

void BaroAccKF::predict(float dt) {
  Eigen::Matrix4d A;
  A << 1,dt,0,0,
                   0,1,dt,-dt,
                   0,0,1,0,
                   0,0,0,1;

  //Prediction Step
  Eigen::Vector4d Xk = A*Xkp;
  Eigen::Matrix4d Pk = A*Pkp*A.transpose()+Qkp;

  Xkp = Xk;
  Pkp = Pk;

  this->x = Xkp(0);
  this->v = Xkp(1);
  this->a = Xkp(2);
  this->b = Xkp(3);
}

void BaroAccKF::updateBaro(float baro) {
  Eigen::Vector4d H; 
  H <<  1,
                       0,
                       0,
                       0;
  H = H.transpose();
  float R = {0.36};
  //update step
  float y = {baro};
  float V = y-(H*Xkp)(0,0);
  float S = (H*Pkp*(H.transpose()))(0,0)+R;

  float S_inv = S;
  bool is_nonsingular = (1/S_inv);
  if (!is_nonsingular) {
    return;
  }
  
  Eigen::Vector4f K = Pkp*(H.transpose())(0,0)*S_inv;
  Xkp = Xkp+K*V;
  Pkp = Pkp-K*S*(K.transpose());

  this->x = Xkp(0);
  this->v = Xkp(1);
  this->a = Xkp(2);
  this->b = Xkp(3);
}

void BaroAccKF::updateAccel(float acc) {

  Eigen::Vector4f H;
  H << 0, 
                       0, 
                       1, 
                       0;
  H = H.transpose();
  float R = {0.001};

  //update step
  float y = {acc};
  float V = y-(H*Xkp)(0,0);
  float S = (H*Pkp*(H.transpose()))(0,0)+R;

  float S_inv = S;
  bool is_nonsingular = (1/S_inv);
  if (!is_nonsingular) {
    return;
  }
  
  Eigen::Vector4d K = Pkp*(H.transpose())*S_inv;
  Xkp = Xkp+K*V;
  Pkp = Pkp-K*S*(K.transpose());

  this->x = Xkp(0);
  this->v = Xkp(1);
  this->a = Xkp(2);
  this->b = Xkp(3);
}
