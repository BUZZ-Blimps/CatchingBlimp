/*
 Kalman_Filter_Tran_Vel_Est.h - Library that will output the flow rate and surface quality content 
*/
#pragma once

#include <Eigen/Dense>
#include "vector"

class Kalman_Filter_Tran_Vel_Est
{
  public:
    Kalman_Filter_Tran_Vel_Est();
    void predict_vel();
    void update_vel_acc(float Ax, float Ay);
    void update_vel_optical(float flow_x, float flow_y);
    Eigen::Matrix<float,8,1> xhat;
    Eigen::Matrix<float,8,8> Phat;
    float x_vel_est = 0;
    float y_vel_est = 0;
    
  private:
    float dt_init_F;
  
};
