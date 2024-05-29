#include "accelGCorrection.h"

AccelGCorrection::AccelGCorrection() {
    // Serial.println("Starting correction");
}

void AccelGCorrection::updateData(float accX, float accY, float accZ, float pitch, float roll) {

    pitch = pitch*3.1415/180;
    roll = roll*3.1415/180;

    //set up rotation matrix
    Matrix<3,3> rPitch = {cos(pitch), 0, sin(pitch),
                          0,1,0,
                          -sin(pitch),0,cos(pitch)};
    Matrix<3,3> rRoll = {1,0,0,
                         0,cos(roll),-sin(roll),
                         0,sin(roll),cos(roll)};

    Matrix<3,3> r = rPitch*rRoll;

    //multiply by gravity and subtract from measurement
    Matrix<3,1> a = {accX*9.81+0.0081, accY*9.81+0.0014, -accZ*9.81+0.1672};
    Matrix<3,1> ac = a-r*g;

    Matrix<3,1> ag = ~r*ac;

    ax = ac(0);
    ay = ac(1);
    az = ac(2);

    agx = ag(0);
    agy = ag(1);
    agz = ag(2);
}