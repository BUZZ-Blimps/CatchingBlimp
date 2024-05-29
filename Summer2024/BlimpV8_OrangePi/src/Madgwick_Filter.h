/*
 Madgwick_Filter.h 
*/

#pragma once
#include "Arduino.h"
#include "vector"

class Madgwick_Filter
{
  public:
    Madgwick_Filter();
    void Madgwick_Update(float gyr_rateXraw, float gyr_rateYraw, float gyr_rateZraw, float AccXraw, float AccYraw, float AccZraw);
    float roll_final;
    float pitch_final;
    float yaw_final;
    float q1;
    float q2;
    float q3;
    float q4;

  private:
    std::vector<float> update_quat(float Gyr_RateX, float Gyr_RateY, float Gyr_RateZ, float AccelX, float AccelY, float AccelZ, float q1_est, float q2_est, float q3_est, float q4_est);
    std::vector<float> get_euler_angles_from_quat(float q1, float q2, float q3, float q4);
    std::vector<float> q_est_orig = {1, 0, 0, 0}; //Assumed initial orientation of IMU
    std::vector<float> q_est_g_lock = {1, 0, 0, 0}; //Another orientation just to account for gymbol lock
    float init_time;
    float t_interval;
};
