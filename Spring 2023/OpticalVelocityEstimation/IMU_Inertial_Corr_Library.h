/*
 IMU_Inertial_Corr_Library.h 
*/

#pragma once
#include "Arduino.h"

class IMU_Inertial_Corr_Library
{
  public:
    IMU_Inertial_Corr_Library();
    void Update_Inertial_Accel_Corr(float Madgwick_Roll, float Madgwick_Pitch, float Madgwick_Yaw, float AccelX, float AccelY, float AccelZ);
    float AccelX_inert_corr;
    float AccelY_inert_corr;
    float AccelZ_inert_corr;

  private:

};
