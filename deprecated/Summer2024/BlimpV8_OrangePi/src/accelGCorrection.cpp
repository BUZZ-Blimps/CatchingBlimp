#include "accelGCorrection.h"

AccelGCorrection::AccelGCorrection() {
    // Serial.println("Starting correction");
    this->g << 0,0,-9.81;
}

void AccelGCorrection::updateData(float accX, float accY, float accZ, float pitch, float roll) {

    pitch = pitch*3.1415/180;
    roll = roll*3.1415/180;

    //set up rotation matrix
    Matrix3f rPitch; rPitch << cos(pitch), 0, sin(pitch),
                          0,1,0,
                          -sin(pitch),0,cos(pitch);

    Matrix3f rRoll; rRoll << 1,0,0,
                         0,cos(roll),-sin(roll),
                         0,sin(roll),cos(roll);

    Matrix3f r = rPitch*rRoll;

    //multiply by gravity and subtract from measurement
    Vector3f a; a<< accX*9.81+0.0081, accY*9.81+0.0014, -accZ*9.81+0.1672;
    Vector3f ac = a-r*g;

    Vector3f ag = r.transpose()*ac;

    ax = ac(0);
    ay = ac(1);
    az = ac(2);

    agx = ag(0);
    agy = ag(1);
    agz = ag(2);
}