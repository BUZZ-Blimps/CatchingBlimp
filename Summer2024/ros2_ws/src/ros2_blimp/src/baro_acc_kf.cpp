#include "baro_acc_kf.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

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
  Eigen::Matrix4f A;
  A << 1,dt,0,0,
                   0,1,dt,-dt,
                   0,0,1,0,
                   0,0,0,1;

  //Prediction Step
  Eigen::Vector4f Xk = A*Xkp;

  Eigen::Matrix4f Pk = A*Pkp*A.transpose()+Qkp;

  Xkp = Xk;
  Pkp = Pk;

  this->x = Xkp(0);
  this->v = Xkp(1);
  this->a = Xkp(2);
  this->b = Xkp(3);
}

void BaroAccKF::updateBaro(float baro) {
  Eigen::RowVector4f H; 
  H <<  1,0,0,0;
  float R = 0.36;

  //update step
  float y = baro;
  float V = y-(H*Xkp);
  float S = H*Pkp*H.transpose()+R;

//////////////////////////////////
/////////// QUESTION /////////////
//////////////////////////////////

  bool is_nonsingular = S;
  if (!is_nonsingular) {
  return;
  }
  float S_inv = 1/S;

//////////////////////////////////
/////////// QUESTION /////////////
//////////////////////////////////  
  
  Eigen::Vector4f K = Pkp*(H.transpose())*S_inv;
  Xkp = Xkp+K*V;
  Pkp = Pkp-K*S*(K.transpose());

  this->x = Xkp(0);
  this->v = Xkp(1);
  this->a = Xkp(2);
  this->b = Xkp(3);
}

void BaroAccKF::updateAccel(float acc) {

  Eigen::RowVector4f H;
  H << 0, 0, 1, 0;

  float R = 0.001;

  //update step
  float y = acc;
  float V = y-(H*Xkp);
  float S = (H*Pkp*(H.transpose()))+R;

  bool is_nonsingular = S;
  if (!is_nonsingular) {
  return;
  }
  float S_inv = 1/S;
  
  Eigen::Vector4f K = Pkp*(H.transpose())*S_inv;
  Xkp = Xkp+K*V;
  Pkp = Pkp-K*S*(K.transpose());

  this->x = Xkp(0);
  this->v = Xkp(1);
  this->a = Xkp(2);
  this->b = Xkp(3);
}
