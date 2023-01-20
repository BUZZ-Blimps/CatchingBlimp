/*
  References
  https://nitinjsanket.github.io/tutorials/attitudeest/madgwick.html
  https://github.com/arduino-libraries/MadgwickAHRS

  The IMU needs to be orientated so that the imu flat with the plugs, labels, and chips on top of the board. The math will need to be modified if flipped.

*/
#include "BerryIMU_v3.h"
#include "Madgwick_Filter.h"

BerryIMU_v3 BerryIMU;
Madgwick_Filter Madgwick_F;

void setup() {
  Serial.begin(115200);  // start serial for output
}

void loop() {

  BerryIMU.IMU_read();
  Madgwick_F.Madgwick_Update(BerryIMU.gyr_rateXraw, BerryIMU.gyr_rateYraw, BerryIMU.gyr_rateZraw, BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw);
  float roll = Madgwick_F.roll_final;
  float pitch = Madgwick_F.pitch_final;
  float yaw = Madgwick_F.yaw_final;
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);
}
