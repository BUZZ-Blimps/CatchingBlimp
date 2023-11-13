/*
  Madgwick_Filter.cpp
*/

#include "Madgwick_Filter.h"
#include "vector"

Madgwick_Filter::Madgwick_Filter() {
  init_time = micros();
}

//Output
void Madgwick_Filter::Madgwick_Update(float gyr_rateXraw, float gyr_rateYraw, float gyr_rateZraw, float AccXraw, float AccYraw, float AccZraw) {
  //Time Interval
  float final_time = micros();
  t_interval = (final_time - init_time) / 1000000; //in seconds
  init_time = micros();
  float gx = gyr_rateXraw;
  float gy = gyr_rateYraw;
  float gz = gyr_rateZraw;

  //Gravity, gyro, and accel quaterions
  std::vector<float> g_W = {0, 0, 0, 1}; //May need to make neg depending on orientation
  std::vector<float> gyro_I = {0, gx * (3.1415 / 180), gy * (3.1415 / 180), gz * (3.1415 / 180)}; // in rad/s(converted from deg/s)
  //std::vector<float> gyro_I = {0, -gy * (3.1415 / 180), gx * (3.1415 / 180), gz * (3.1415 / 180)}; // in rad/s(converted from deg/s) for changed coordniates

  float ax = AccXraw;
  float ay = AccYraw;
  float az = AccZraw;
  float mag_accel = sqrtf(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
  std::vector<float> a_I = {0, ax / mag_accel, ay / mag_accel, az / mag_accel}; //Normalized Accel
  //std::vector<float> a_I = {0, -ay / mag_accel, ax / mag_accel, az / mag_accel}; //Normalized Accel for changed cordinates

  std::vector<float> angles_quat_update_orig = update_quat(gyro_I[1], gyro_I[2], gyro_I[3], a_I[1], a_I[2], a_I[3], q_est_orig[0], q_est_orig[1], q_est_orig[2], q_est_orig[3]);
  std::vector<float> angles_quat_update_g_lock = update_quat(gyro_I[2], gyro_I[1], -gyro_I[3], -a_I[2], -a_I[1], a_I[3], q_est_g_lock[0], q_est_g_lock[1], q_est_g_lock[2], q_est_g_lock[3]);

  q_est_orig = {angles_quat_update_orig[0], angles_quat_update_orig[1], angles_quat_update_orig[2], angles_quat_update_orig[3]};
  q_est_g_lock = {angles_quat_update_g_lock[0], angles_quat_update_g_lock[1], angles_quat_update_g_lock[2], angles_quat_update_g_lock[3]};

  //Convert quat to euler angles for orig config
  std::vector<float> angles_euler_orig = get_euler_angles_from_quat(q_est_orig[0], q_est_orig[1], q_est_orig[2], q_est_orig[3]);
  float roll_orig = angles_euler_orig[0]; //converted to degrees //x-axis rot
  float pitch_orig = angles_euler_orig[1]; //converted to degrees //y-axis rot
  float yaw_orig = angles_euler_orig[2]; //converted to degrees   //z-axis rot

  //Convert quat to euler angles for g lock config
  std::vector<float> angles_euler_g_lock = get_euler_angles_from_quat(q_est_g_lock[0], q_est_g_lock[1], q_est_g_lock[2], q_est_g_lock[3]);
  float roll_g_lock = angles_euler_g_lock[0]; //converted to degrees //y-axis rot
  float pitch_new = roll_g_lock;
  float pitch_g_lock = angles_euler_g_lock[1]; //converted to degrees //x-axis rot
  float roll_new = pitch_g_lock;
  float yaw_g_lock = angles_euler_g_lock[2]; //converted to degrees   //z-axis rot
  //Stick with origional yaw (maybe try averaging it)

  //Fuse these to get the best of both configs for the final roll, pitch, and yaw
  if (abs(pitch_orig) >= 45) {
    roll_final = roll_new;
    pitch_final = pitch_new;
  }
  else {
    roll_final = roll_orig;
    pitch_final = pitch_orig;
  }
  
  q1 = q_est_orig[0];
  q2 = q_est_orig[1];
  q3 = q_est_orig[2];
  q4 = q_est_orig[3];
  yaw_final = yaw_orig;

  //  Serial.print(roll_orig);
  //  Serial.print(",");
  //  Serial.print(pitch_orig);
  //  Serial.print(",");
  //  Serial.println(yaw_orig);
  //  Serial.print(roll_g_lock);
  //  Serial.print(",");
  //  Serial.print(pitch_g_lock);
  //  Serial.print(",");
  //  Serial.println(yaw_g_lock);
  //  Serial.print(roll_final);
  //  Serial.print(",");
  //  Serial.print(pitch_final);
  //  Serial.print(",");
  //  Serial.println(yaw_final);
  //  Serial.print(q_est_orig[0]);
  //  Serial.print(",");
  //  Serial.print(q_est_orig[1]);
  //  Serial.print(",");
  //  Serial.print(q_est_orig[2]);
  //  Serial.print(",");
  //  Serial.println(q_est_orig[3]);
  //  Serial.print(q_est_g_lock[0]);
  //  Serial.print(",");
  //  Serial.print(q_est_g_lock[1]);
  //  Serial.print(",");
  //  Serial.print(q_est_g_lock[2]);
  //  Serial.print(",");
  //  Serial.println(q_est_g_lock[3]);
  //Serial.println("Next");

  //Prints frequency of the filter
  //Serial.println(1 / t_interval); //Frequency of the filter
  //delay(10);
}

std::vector<float> Madgwick_Filter::update_quat(float Gyr_RateX, float Gyr_RateY, float Gyr_RateZ, float AccelX, float AccelY, float AccelZ, float q1_est, float q2_est, float q3_est, float q4_est) {

  std::vector<float> q_est = {q1_est, q2_est, q3_est, q4_est};
  std::vector<float> gyro_I = {0, Gyr_RateX, Gyr_RateY, Gyr_RateZ};
  std::vector<float> a_I = {0, AccelX, AccelY, AccelZ};

  //q_est components
  float q1 = q_est[0];
  float q2 = q_est[1];
  float q3 = q_est[2];
  float q4 = q_est[3];

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _4q1 = 4.0f * q1;
  float _4q2 = 4.0f * q2;
  float _4q3 = 4.0f * q3;
  float _8q2 = 8.0f * q2;
  float _8q3 = 8.0f * q3;
  float q1q1 = q1 * q1;
  float q2q2 = q2 * q2;
  float q3q3 = q3 * q3;
  float q4q4 = q4 * q4;

  //Update Term
  // Gradient decent algorithm corrective step (del_f)
  float del_f1 = _4q1 * q3q3 + _2q3 * a_I[1] + _4q1 * q2q2 - _2q2 * a_I[2];
  float del_f2 = _4q2 * q4q4 - _2q4 * a_I[1] + 4.0f * q1q1 * q2 - _2q1 * a_I[2] - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * a_I[3];
  float del_f3 = 4.0f * q1q1 * q3 + _2q1 * a_I[1] + _4q3 * q4q4 - _2q4 * a_I[2] - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * a_I[3];
  float del_f4 = 4.0f * q2q2 * q4 - _2q2 * a_I[1] + 4.0f * q3q3 * q4 - _2q3 * a_I[2];

  //Tunable parameter
  float beta = .033; //from the original article

  float del_f_norm = sqrtf(pow(del_f1, 2) + pow(del_f2, 2) + pow(del_f3, 2) + pow(del_f4, 2));
  std::vector<float> del_q_est = { -beta*(del_f1 / del_f_norm),
                                   -beta*(del_f2 / del_f_norm),
                                   -beta*(del_f3 / del_f_norm),
                                   -beta*(del_f4 / del_f_norm)
                                 }; //4x1 Matrix

  //Orientation from Gyroscope
  //quaternion product
  // Rate of change of quaternion from gyroscope
  std::vector<float> q_dot_w = {q_est[0]*gyro_I[0] - q_est[1]*gyro_I[1] - q_est[2]*gyro_I[2] - q_est[3]*gyro_I[3],
                                q_est[0]*gyro_I[1] + q_est[1]*gyro_I[0] + q_est[2]*gyro_I[3] - q_est[3]*gyro_I[2],
                                q_est[0]*gyro_I[2] - q_est[1]*gyro_I[3] + q_est[2]*gyro_I[0] + q_est[3]*gyro_I[1],
                                q_est[0]*gyro_I[3] + q_est[1]*gyro_I[2] - q_est[2]*gyro_I[1] + q_est[3]*gyro_I[0]
                               }; //4x1 Matrix
  q_dot_w = {0.5f * q_dot_w[0],
             0.5f * q_dot_w[1],
             0.5f * q_dot_w[2],
             0.5f * q_dot_w[3]
            };

  //Fuse Measurements
  std::vector<float> q_est_dot = {q_dot_w[0] + del_q_est[0],
                                  q_dot_w[1] + del_q_est[1],
                                  q_dot_w[2] + del_q_est[2],
                                  q_dot_w[3] + del_q_est[3]
                                 };

  q_est = {q_est[0] + q_est_dot[0]*t_interval,
           q_est[1] + q_est_dot[1]*t_interval,
           q_est[2] + q_est_dot[2]*t_interval,
           q_est[3] + q_est_dot[3]*t_interval
          };  //Current estimate


  float q_mag = sqrt(pow(q_est[0], 2) + pow(q_est[1], 2) + pow(q_est[2], 2) + pow(q_est[3], 2));
  std::vector<float> q_norm = {q_est[0] / q_mag,
                               q_est[1] / q_mag,
                               q_est[2] / q_mag,
                               q_est[3] / q_mag
                              };//normalize the quaternion before next iteration
  //will drift if not normalized

  q_est = q_norm; //Update the previous estimate

  std::vector<float> angles_quat_update;
  angles_quat_update.push_back(q_est[0]);
  angles_quat_update.push_back(q_est[1]);
  angles_quat_update.push_back(q_est[2]);
  angles_quat_update.push_back(q_est[3]);
  return angles_quat_update;
}

std::vector<float> Madgwick_Filter::get_euler_angles_from_quat(float q1, float q2, float q3, float q4) {
  float roll_rad = atan2f(q1 * q2 + q3 * q4, 0.5f - q2 * q2 - q3 * q3);
  float roll_deg = roll_rad * (180 / 3.1415);
  float pitch_rad = asinf(-2.0f * (q2 * q4 - q1 * q3));
  float pitch_deg = pitch_rad * (180 / 3.1415);
  float yaw_rad = atan2f(q2 * q3 + q1 * q4, 0.5f - q3 * q3 - q4 * q4);
  float yaw_deg = yaw_rad * (180 / 3.1415);
  std::vector<float> angles_euler;
  angles_euler.push_back(roll_deg);
  angles_euler.push_back(pitch_deg);
  angles_euler.push_back(yaw_deg);
  return angles_euler;
}
