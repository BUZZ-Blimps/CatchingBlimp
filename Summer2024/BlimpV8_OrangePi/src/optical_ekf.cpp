#include "optical_ekf.h"

OpticalEKF::OpticalEKF(float a_new, float b1_new, float b2_new) {
    this->a = a_new;
    this->b1 = b1_new;
    this->b2 = b2_new;

    this->Xkp << 0,
               0,
               0,
               0,
               0,
               0,
               0;
               
  this->Pkp << 1,0,0,0,0,0,0,
                0,1,0,0,0,0,0,
                0,0,1,0,0,0,0,
                0,0,0,1,0,0,0,
                0,0,0,0,1,0,0,
                0,0,0,0,0,1,0,
                0,0,0,0,0,0,1;
               
  this->Qkp << 0.001,0,0,0,0,0,0,
               0,0.00001,0,0,0,0,0, //next three diagonal components must be equal or else b1, and b2 will not be correct
               0,0,0.00001,0,0,0,0,
               0,0,0,0.00001,0,0,0,
               0,0,0,0,0.0000001,0,0,
               0,0,0,0,0,0.01,0,
               0,0,0,0,0,0,0;

}

void OpticalEKF::predict(float dt) {
    Eigen::MatrixXf Fj(7,7);
    Fj << 1,0,0,0,0,0,0,
                   0,1,0,0,0,0,0,
                   0,0,1,0,0,0,0,
                   0,0,0,1,0,0,0,
                   a*(Xkp(1)+Xkp(2)*b1+Xkp(3)*b2),a*Xkp(0),a*Xkp(0)*b1,a*Xkp(0)*b2,0,dt,-dt,
                   0,0,0,0,0,1,0,
                   0,0,0,0,0,0,1;

    //Prediction Step
    Eigen::MatrixXf Xk(7,1);
    Xk << Xkp(0),
                      Xkp(1),
                      Xkp(2),
                      Xkp(3),
                      a*Xkp(0)*(Xkp(1)+Xkp(2)*b1+Xkp(3)*b2)+Xkp(5)*dt-Xkp(6)*dt,
                      Xkp(5),
                      Xkp(6);

    Eigen::MatrixXf Pk(7,7);
    Pk << Fj*Pkp*Fj.transpose()+Qkp;

    Xkp = Xk;
    Pkp = Pk;

    this->z = Xkp(0);
    this->opt = Xkp(1);
    this->gyrox = Xkp(2);
    this->gyroz = Xkp(3);
    this->v = Xkp(4);
    this->ax = Xkp(5);
    this->b = Xkp(6);
}

void OpticalEKF::updateBaro(float baro) {
    Eigen::MatrixXf H(1,7);
    H << 1,0,0,0,0,0,0;
    float R = baroR;   

    //update step
    float y = baro;
    float V = y-(H*Xkp)(0,0);
    float S = (H*Pkp*H.transpose())(0,0)+R;

    bool is_nonsingular = S;
    if (!is_nonsingular) {
    return;
    }
    float S_inv = 1/S;

    Eigen::MatrixXf K(7,1);
    K << Pkp*H.transpose()*S_inv;
    Xkp = Xkp+K*V;
    Pkp = Pkp-K*S*K.transpose();
}

void OpticalEKF::updateOptical(float optical) {
    Eigen::MatrixXf H(1,7);
    H << 0,1,0,0,0,0,0;
    float R = opticalR;   

    //update step
    float y = optical;
    float V = y-(H*Xkp)(0,0);
    float S = (H*Pkp*H.transpose())(0,0)+R;

    bool is_nonsingular = S;
    if (!is_nonsingular) {
    return;
    }
    float S_inv = 1/S;

    Eigen::MatrixXf K(7,1);
    K << Pkp*H.transpose()*S_inv;
    Xkp = Xkp+K*V;
    Pkp = Pkp-K*S*K.transpose();
}

void OpticalEKF::updateGyroX(float gyrox) {
    Eigen::MatrixXf H(1,7);
    H << 0,0,1,0,0,0,0;
    float R = gyroXR;   

    //update step
    float y = gyrox;
    float V = y-(H*Xkp)(0,0);
    float S = (H*Pkp*H.transpose())(0,0)+R;

    bool is_nonsingular = S;
    if (!is_nonsingular) {
    return;
    }
    float S_inv = 1/S;

    Eigen::MatrixXf K(7,1);
    K << Pkp*H.transpose()*S_inv;
    Xkp = Xkp+K*V;
    Pkp = Pkp-K*S*K.transpose();
}

void OpticalEKF::updateGyroZ(float gyroz) {
    Eigen::MatrixXf H(1,7);
    H << 0,0,0,1,0,0,0;
    float R = gyroZR;   

    //update step
    float y = gyroz;
    float V = y-(H*Xkp)(0,0);
    float S = (H*Pkp*H.transpose())(0,0)+R;

    bool is_nonsingular = S;
    if (!is_nonsingular) {
    return;
    }
    float S_inv = 1/S;

    Eigen::MatrixXf K(7,1);
    K = Pkp*H.transpose()*S_inv;
    Xkp = Xkp+K*V;
    Pkp = Pkp-K*S*K.transpose();
}

void OpticalEKF::updateAccelx(float accx) {
    Eigen::MatrixXf H(1,7);
    H << 0,0,0,0,0,1,0;
    float R = accelXR;   

    //update step
    float y = accx;
    float V = y-(H*Xkp)(0,0);
    float S = (H*Pkp*H.transpose())(0,0)+R;

    bool is_nonsingular = S;
    if (!is_nonsingular) {
    return;
    }
    float S_inv = 1/S;

    Eigen::MatrixXf K(7,1);
    K << Pkp*H.transpose()*S_inv;
    Xkp = Xkp+K*V;
    Pkp = Pkp-K*S*K.transpose();
}