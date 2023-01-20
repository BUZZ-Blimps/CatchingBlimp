#include "Kalman_Filter_Tran_Vel_Est.h"
#include "Optical_Flow.h"
#include "BerryIMU_v3.h"
#include "Madgwick_Filter.h"
#include "IMU_Inertial_Corr_Library.h"

using namespace BLA;
float optical_initial;

Kalman_Filter_Tran_Vel_Est kal_vel;
Optical_Flow Flow;
BerryIMU_v3 BerryIMU;
Madgwick_Filter Madgwick_F;
IMU_Inertial_Corr_Library Inertial_Corr;

volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;
const byte interruptPin = 22;
float Z_distance_m = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), Pulse, CHANGE);

  optical_initial = micros();
}

void loop() {

  BerryIMU.IMU_read();
  Madgwick_F.Madgwick_Update(BerryIMU.gyr_rateXraw, BerryIMU.gyr_rateYraw, BerryIMU.gyr_rateZraw, BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw);
  Inertial_Corr.Update_Inertial_Accel_Corr(Madgwick_F.roll_final, Madgwick_F.pitch_final, Madgwick_F.yaw_final, BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw);
  Flow.read_buffer();

  kal_vel.predict_vel();
  kal_vel.update_vel_acc(Inertial_Corr.AccelX_inert_corr, Inertial_Corr.AccelY_inert_corr);
//  Serial.print(Inertial_Corr.AccelX_inert_corr);
//  Serial.print(",");
//  Serial.println(Inertial_Corr.AccelY_inert_corr);

  //Update Optical Flow
  float optical_now = micros();
  if (optical_now - optical_initial >= 15152) { //Update every #Hz
    optical_initial = optical_now;

    Flow.update_flow(BerryIMU.gyr_rateXraw, BerryIMU.gyr_rateYraw, Z_distance_m);
    kal_vel.update_vel_optical(-Flow.x_motion_comp, Flow.y_motion_comp); 
    //Will need to check that flow velocity direction matches acceleration direction
    Serial.print(-Flow.x_motion_comp);
    //Serial.print(",");
    Serial.println(Flow.y_motion_comp);
  }

  float x_tran_vel = kal_vel.x_vel_est;
  float y_tran_vel = kal_vel.y_vel_est;

  //***********************************Will Continue to output zeros untill the Q and R matricies are not 0 matricies********************************
  //Serial.print(x_tran_vel);
  //Serial.print(",");
  //Serial.println(y_tran_vel);

  delay(10);

  if (newPulseDurationAvailable) {
    newPulseDurationAvailable = false;
    unsigned long pulseDuration = pulseInTimeEnd - pulseInTimeBegin;

    //Long Range Sensor
    float Z_distance_ft = pulseDuration * 0.0328084; //In feet
    Z_distance_m = Z_distance_ft * 0.3048; //in meters
    //Serial.println(Z_distance_m);
  }
}

void Pulse() {
  if (digitalRead(interruptPin) == HIGH) {
    // start measuring
    pulseInTimeBegin = micros();
  }
  else {
    // stop measuring
    pulseInTimeEnd = micros();
    newPulseDurationAvailable = true;
  }
}
