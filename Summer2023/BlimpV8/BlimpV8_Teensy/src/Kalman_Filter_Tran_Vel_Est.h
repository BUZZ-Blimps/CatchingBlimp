/*
 Kalman_Filter_Tran_Vel_Est.h - Library that will output the flow rate and surface quality content 
*/
#pragma once

#include "Arduino.h"
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include "vector"

using namespace BLA;

class Kalman_Filter_Tran_Vel_Est
{
  public:
    Kalman_Filter_Tran_Vel_Est();
    void predict_vel();
    void update_vel_acc(float Ax, float Ay);
    void update_vel_optical(float flow_x, float flow_y);
    BLA::Matrix<8, 1> xhat;
    BLA::Matrix<8, 8> Phat;
    float x_vel_est = 0;
    float y_vel_est = 0;
    
  private:
    float dt_init_F;
  
};
