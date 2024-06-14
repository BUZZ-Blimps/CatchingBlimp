#include "gyro_ekf.h"

GyroEKF::GyroEKF() {
    this->Qkp <<0.000001,0,0,0,0,0,0,0,0,
                0,0.000001,0,0,0,0,0,0,0,
                0,0,0.000001,0,0,0,0,0,0,
                0,0,0,0.0001,0,0,0,0,0,
                0,0,0,0,0.0001,0,0,0,0,
                0,0,0,0,0,0.0001,0,0,0,
                0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0;

    this->Pkp <<1,0,0,0,0,0,0,0,0,
                0,1,0,0,0,0,0,0,0,
                0,0,1,0,0,0,0,0,0,
                0,0,0,1,0,0,0,0,0,
                0,0,0,0,1,0,0,0,0,
                0,0,0,0,0,1,0,0,0,
                0,0,0,0,0,0,1,0,0,
                0,0,0,0,0,0,0,1,0,
                0,0,0,0,0,0,0,0,1;

    this->Xkp <<0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0;
}

void GyroEKF::predict(float dt) {
    VectorXf Xk(9); Xk <<Xkp(0) + (cos(Xkp(1))*(Xkp(3)-Xkp(6))+sin(Xkp(1))*(Xkp(5)-Xkp(8)))*dt,
        Xkp(1) + (Xkp(4)-Xkp(7))*dt,
        Xkp(2) + (-cos(Xkp(1))*(Xkp(3)-Xkp(6))+sin(Xkp(1))*(Xkp(4)-Xkp(7))+cos(Xkp(0))*cos(Xkp(1))*(Xkp(5)-Xkp(8)))*dt,
        Xkp(3),
        Xkp(4),
        Xkp(5),
        Xkp(6),
        Xkp(7),
        Xkp(8);

    MatrixXf Fj(9,9); Fj<<1,(-sin(Xkp(1))*(Xkp(3)-Xkp(6))+cos(Xkp(1))*(Xkp(5)-Xkp(8)))*dt,0,cos(Xkp(1))*dt,0,sin(Xkp(1))*dt,-cos(Xkp(1))*dt,0,-sin(Xkp(1)*dt),
                      0,1,0,0,dt,0,0,-dt,0,
                      (-sin(Xkp(0))*cos(Xkp(1))*(Xkp(5)-Xkp(8)))*dt,(sin(Xkp(1))*(Xkp(3)-Xkp(6))+cos(Xkp(1))*(Xkp(4)-Xkp(7))-cos(Xkp(0))*sin(Xkp(1))*(Xkp(5)-Xkp(8)))*dt,1,-cos(Xkp(1))*dt,sin(Xkp(1))*dt,cos(Xkp(0))*cos(Xkp(1))*dt,cos(Xkp(1))*dt,-sin(Xkp(1))*dt,-cos(Xkp(0))*cos(Xkp(1))*dt,
                      0,0,0,1,0,0,0,0,0,
                      0,0,0,0,1,0,0,0,0,
                      0,0,0,0,0,1,0,0,0,
                      0,0,0,0,0,0,1,0,0,
                      0,0,0,0,0,0,0,1,0,
                      0,0,0,0,0,0,0,0,1;

    MatrixXf Pk(9,9); Pk = Fj*Pkp*Fj.transpose()+Qkp;
    Xkp = Xk;
    Pkp = Pk;
    
    roll = Xkp(0);
    pitch = Xkp(1);
    yaw = Xkp(2);
    rollRate = Xkp(3);
    pitchRate = Xkp(4);
    yawRate = Xkp(5);
    rollRateB = Xkp(6);
    pitchRateB = Xkp(7);
    yawRateB = Xkp(8);
}

void GyroEKF::updateGyro(float gyrox, float gyroy, float gyroz) {

    MatrixXf H(3,9); H<<0,0,0,1,0,0,0,0,0,
                     0,0,0,0,1,0,0,0,0,
                     0,0,0,0,0,1,0,0,0;


    float r = 0.01;

    Matrix3f R; R<<r,0,0,
                     0,r,0,
                     0,0,r;

    //update step
    Vector3f y; y<<gyrox, gyroy, gyroz;
    Vector3f V = y-H*Xkp;
    Matrix<float, 3, 3> S = H*Pkp*H.transpose()+R;

    Eigen::FullPivLU<Eigen::Matrix<float, 3, 3>> lu(S);
    bool is_nonsingular = lu.isInvertible(); // not actually inverting S??
    if (!is_nonsingular) {
    return;
    }

    MatrixXf K(9,3); K = Pkp*H.transpose()*S.inverse();
    Xkp = Xkp+K*V;
    Pkp = Pkp-K*S*K.transpose();   
}

void GyroEKF::updateAccel(float accx, float accy, float accz) {
    
    //compute angle for the angle update
    float phi = atan2(accy, sqrt(accx*accx+accz*accz));
    float theta = atan2(accx, sqrt(accy*accy+accz*accz));

    MatrixXf H(2,9); H<<1,0,0,0,0,0,0,0,0,
                     0,1,0,0,0,0,0,0,0;

    float r = 0.01;

    Matrix2f R; R<<r,0,
                     0,r;

    //update step
    Vector2f y; y<<phi,theta;
    Vector2f V = y-H*Xkp;
    Matrix2f S = H*Pkp*H.transpose()+R;

    FullPivLU<Matrix<float, 2, 2>> lu(S);
    bool is_nonsingular = lu.isInvertible();
    if (!is_nonsingular) {
    return;
    }


    MatrixXf K(9,2); K = Pkp*H.transpose()*S.inverse();
    Xkp = Xkp+K*V;
    Pkp = Pkp-K*S*K.transpose();   
}