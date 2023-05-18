/*
  Kalman_Filter_Tran_Vel_Est.cpp - Library that will output the flow rate and surface quality content
*/

#include "Arduino.h"
#include "Kalman_Filter_Tran_Vel_Est.h"
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include "vector"

using namespace BLA;

Kalman_Filter_Tran_Vel_Est::Kalman_Filter_Tran_Vel_Est() {
  // xhat = {x, y, xdot, ydot, xddot, yddot, xbias, ybias}
  BLA::Matrix<8, 1> xhat = {0,0,0,0,0,0,0,0}; // 8X1 Matrix
  BLA::Matrix<8, 8> Phat = {0,0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,0}; // 8X8 Matrix
  dt_init_F = micros();
}

void Kalman_Filter_Tran_Vel_Est::predict_vel(){
  //Process Noise Q is a 8X8 Matrix
  //Come back and finish
  BLA::Matrix<8, 8> Q = {0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,
                         0,0,0.001,0,0,0,0,0,
                         0,0,0,0.001,0,0,0,0,
                         0,0,0,0,0.01,0,0,0,
                         0,0,0,0,0,.01,0,0,
                         0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0};

  float dt_now_F = micros();
  float dt = (dt_now_F - dt_init_F)/1000000; //In seconds
  //Serial.println(dt);
  dt_init_F = dt_now_F;

  // F is a 8X8 Matrix
  BLA::Matrix<8, 8> F = {1,0,dt,0,0.5*dt*dt,0,-0.5*dt*dt,0,
                         0,1,0,dt,0,0.5*dt*dt,0,-0.5*dt*dt,
                         0,0,1,0,dt,0,-dt,0,
                         0,0,0,1,0,dt,0,-dt,
                         0,0,0,0,1,0,-1,0,
                         0,0,0,0,0,1,0,-1,
                         0,0,0,0,0,0,1,0,
                         0,0,0,0,0,0,0,1};
  BLA::Matrix<8, 8> F_T = ~F;
  
  //Predict
  xhat = F*xhat;
  Phat = F*Phat*F_T+Q;
  //Serial << "Matrix: " << xhat << '\n';

  //Outputs
  x_vel_est = xhat(2);
  y_vel_est = xhat(3);
}

void Kalman_Filter_Tran_Vel_Est::update_vel_acc(float Ax, float Ay){
  // H is a 2X8 Matrix
  // Takes in measurments from the x/y accelerations
  BLA::Matrix<2, 8> H_accel = {0,0,0,0,1,0,0,0,
                               0,0,0,0,0,1,0,0};
  BLA::Matrix<8, 2> H_accel_T = ~H_accel;
  
  // Zn is a 2X1 Matrix                       
  BLA::Matrix<2, 1> Zn_accel = {Ax*9.81,
                                Ay*9.81}; //Actual Ax and Ay values
                                          //Converts from G's to m/s^2

  //Measurement Uncertainty R
  BLA::Matrix<2, 2> Rn_accel = {0.00001,0,
                                0,0.00001};

  // Kn is a 8X2 Matrix (Kalman Gain)
  BLA::Matrix<2, 2> HPG_TR_accel = H_accel*Phat*H_accel_T+Rn_accel;
  bool is_nonsingular = Invert(HPG_TR_accel); 
  if (!is_nonsingular){
    return;
  }
  //Invert changes the value internally and outputs a boolean
  BLA::Matrix<8, 2> Kn_accel = Phat*H_accel_T*HPG_TR_accel;

  //xhat and Phat accel
  xhat = xhat+(Kn_accel*(Zn_accel-H_accel*xhat));
  BLA::Matrix<8, 8> I_accel = {1,0,0,0,0,0,0,0,
                               0,1,0,0,0,0,0,0,
                               0,0,1,0,0,0,0,0,
                               0,0,0,1,0,0,0,0,
                               0,0,0,0,1,0,0,0,
                               0,0,0,0,0,1,0,0,
                               0,0,0,0,0,0,1,0,
                               0,0,0,0,0,0,0,1};
  BLA::Matrix<8, 8> IKnH_accel = I_accel-Kn_accel*H_accel;  
  BLA::Matrix<8, 8> IKnH_accel_T = ~IKnH_accel;    
  BLA::Matrix<2, 8> Kn_accel_T = ~Kn_accel;              
  Phat = IKnH_accel*Phat*IKnH_accel_T+Kn_accel*Rn_accel*Kn_accel_T;
  //Serial << "Matrix: " << HPG_TR_accel << '\n';
  //Serial << "Matrix: " << invert_accel << '\n';

  //Outputs
  x_vel_est = xhat(2);
  y_vel_est = xhat(3);
}

void Kalman_Filter_Tran_Vel_Est::update_vel_optical(float flow_x, float flow_y){
  // H is a 2X8 Matrix
  // Takes in measurments from the x/y velocities
  BLA::Matrix<2, 8> H_optical = {0,0,1,0,0,0,0,0,
                                 0,0,0,1,0,0,0,0};
  BLA::Matrix<8, 2> H_optical_T = ~H_optical;
  
  // Zn is a 2X1 Matrix                       
  BLA::Matrix<2, 1> Zn_optical = {flow_x,
                                  flow_y}; //Actual flow velocities

  //Measurement Uncertainty R
  BLA::Matrix<2, 2> Rn_optical = {0.6,0,
                                  0,0.6};

  // Kn is a 8X2 Matrix (Kalman Gain)
  BLA::Matrix<2, 2> HPG_TR_optical = H_optical*Phat*H_optical_T+Rn_optical;
  bool is_nonsingular = Invert(HPG_TR_optical);
  if (!is_nonsingular){
    return;
  } 
  //Invert changes the value internally and outputs a boolean
  BLA::Matrix<8, 2> Kn_optical = Phat*H_optical_T*HPG_TR_optical;

  //xhat and Phat optical
  xhat = xhat+(Kn_optical*(Zn_optical-H_optical*xhat));
  BLA::Matrix<8, 8> I_optical = {1,0,0,0,0,0,0,0,
                                 0,1,0,0,0,0,0,0,
                                 0,0,1,0,0,0,0,0,
                                 0,0,0,1,0,0,0,0,
                                 0,0,0,0,1,0,0,0,
                                 0,0,0,0,0,1,0,0,
                                 0,0,0,0,0,0,1,0,
                                 0,0,0,0,0,0,0,1};
  BLA::Matrix<8, 8> IKnH_optical = I_optical-Kn_optical*H_optical;  
  BLA::Matrix<8, 8> IKnH_optical_T = ~IKnH_optical;    
  BLA::Matrix<2, 8> Kn_optical_T = ~Kn_optical;              
  Phat = IKnH_optical*Phat*IKnH_optical_T+Kn_optical*Rn_optical*Kn_optical_T;
  //Serial << "Matrix: " << xhat << '\n';

  //Outputs
  x_vel_est = xhat(2);
  y_vel_est = xhat(3);
}
