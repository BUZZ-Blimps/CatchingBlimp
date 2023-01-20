/*
  BerryIMU_v3.cpp
*/

#include "BerryIMU_v3.h"
#include "IMU_Inertial_Corr_Library.h"
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

IMU_Inertial_Corr_Library::IMU_Inertial_Corr_Library() {

}

void IMU_Inertial_Corr_Library::Update_Inertial_Accel_Corr(float Madgwick_Roll, float Madgwick_Pitch, float Madgwick_Yaw, float AccelX, float AccelY, float AccelZ) {
  double pi = 3.14159265;
  float roll = Madgwick_Roll * pi / 180; //converted to radians around X axis
  float pitch = Madgwick_Pitch * pi / 180; //converted to radians around Y axis
  float yaw = Madgwick_Yaw * pi / 180; //converted to radians around Z axis

  //Convert to find the acceleration in the earth frame from the local frame using Rotation matrix
  //https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  BLA::Matrix<3, 3> rot_from_inert_to_earth = {cos(pitch)*cos(yaw), -1.0f * cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw), sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw),
                                               cos(pitch)*sin(yaw), cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw), -1.0f * sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw),
                                               -1.0f * sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch)};
  BLA::Matrix<3, 1> inert_accel = {AccelX,
                                   AccelY,
                                   AccelZ};
  BLA::Matrix<3, 1> earth_accel = rot_from_inert_to_earth * inert_accel;

  float AccelX_earth = earth_accel(0);
  float AccelY_earth = earth_accel(1);
  float AccelZ_earth = earth_accel(2);

  //Converts the earth frame back to the inertia frame after subtracting the gravitational acceleration
  BLA::Matrix<3, 1> earth_accel_mod = {earth_accel(0),
                                       earth_accel(1),
                                       earth_accel(2) - 1.0f}; //Subtract off gravity component
  BLA::Matrix<3, 3> rot_from_earth_to_inert = ~rot_from_inert_to_earth; //Transpose of rot_from_inert_to_earth matrix
  BLA::Matrix<3, 1> inert_accel_corrected = rot_from_earth_to_inert * earth_accel_mod;

  AccelX_inert_corr = inert_accel_corrected(0);
  AccelY_inert_corr = inert_accel_corrected(1);
  AccelZ_inert_corr = inert_accel_corrected(2);

//    Serial.print("Inertial Acceleration: ");
//    Serial.print(AccelX);
//    Serial.print(",");
//    Serial.print(AccelY);
//    Serial.print(",");
//    Serial.println(AccelZ);
//    Serial.print("   Earth Acceleration: ");
//    Serial.print(AccelX_earth);
//    Serial.print(",");
//    Serial.print(AccelY_earth);
//    Serial.print(",");
//    Serial.println(AccelZ_earth);
//    Serial.print("Inertial Acceleration Corrected: ");
//    Serial.print(AccelX_inert_corr);
//    Serial.print(",");
//    Serial.print(AccelY_inert_corr);
//    Serial.print(",");
//    Serial.println(AccelZ_inert_corr);
}
