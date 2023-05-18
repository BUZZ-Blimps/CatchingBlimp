#include "optical_ekf.h"

OpticalEKF::OpticalEKF(float a_new, float b1_new, float b2_new) {
    this->a = a_new;
    this->b1 = b1_new;
    this->b2 = b2_new;

    this->Xkp = {0,
               0,
               0,
               0,
               0,
               0,
               0};
               
  this->Pkp = {1,0,0,0,0,0,0,
                0,1,0,0,0,0,0,
                0,0,1,0,0,0,0,
                0,0,0,1,0,0,0,
                0,0,0,0,1,0,0,
                0,0,0,0,0,1,0,
                0,0,0,0,0,0,1};
               
  this->Qkp = {0.001,0,0,0,0,0,0,
               0,0.00001,0,0,0,0,0, //next three diagonal components must be equal or else b1, and b2 will not be correct
               0,0,0.00001,0,0,0,0,
               0,0,0,0.00001,0,0,0,
               0,0,0,0,0.0000001,0,0,
               0,0,0,0,0,0.01,0,
               0,0,0,0,0,0,0};

}

void OpticalEKF::predict(float dt) {
    Matrix<7,7> Fj = {1,0,0,0,0,0,0,
                   0,1,0,0,0,0,0,
                   0,0,1,0,0,0,0,
                   0,0,0,1,0,0,0,
                   a*(Xkp(1)+Xkp(2)*b1+Xkp(3)*b2),a*Xkp(0),a*Xkp(0)*b1,a*Xkp(0)*b2,0,dt,-dt,
                   0,0,0,0,0,1,0,
                   0,0,0,0,0,0,1};

    //Prediction Step
    Matrix<7,1> Xk = {Xkp(0),
                      Xkp(1),
                      Xkp(2),
                      Xkp(3),
                      a*Xkp(0)*(Xkp(1)+Xkp(2)*b1+Xkp(3)*b2)+Xkp(5)*dt-Xkp(6)*dt,
                      Xkp(5),
                      Xkp(6)};

    Matrix<7,7> Pk = Fj*Pkp*~Fj+Qkp;

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
    Matrix<1,7> H = {1,0,0,0,0,0,0};
    Matrix<1,1> R = {baroR};   

    //update step
    Matrix<1,1> y = {baro};
    Matrix<1,1> V = y-H*Xkp;
    Matrix<1,1> S = H*Pkp*~H+R;

    Matrix<1,1> S_inv = S;
    bool is_nonsingular = Invert(S_inv);
    if (!is_nonsingular) {
    return;
    }

    Matrix<7,1> K = Pkp*~H*S_inv;
    Xkp = Xkp+K*V;
    Pkp = Pkp-K*S*~K;
}

void OpticalEKF::updateOptical(float optical) {
    Matrix<1,7> H = {0,1,0,0,0,0,0};
    Matrix<1,1> R = {opticalR};   

    //update step
    Matrix<1,1> y = {optical};
    Matrix<1,1> V = y-H*Xkp;
    Matrix<1,1> S = H*Pkp*~H+R;

    Matrix<1,1> S_inv = S;
    bool is_nonsingular = Invert(S_inv);
    if (!is_nonsingular) {
    return;
    }

    Matrix<7,1> K = Pkp*~H*S_inv;
    Xkp = Xkp+K*V;
    Pkp = Pkp-K*S*~K;
}

void OpticalEKF::updateGyroX(float gyrox) {
    Matrix<1,7> H = {0,0,1,0,0,0,0};
    Matrix<1,1> R = {gyroXR};   

    //update step
    Matrix<1,1> y = {gyrox};
    Matrix<1,1> V = y-H*Xkp;
    Matrix<1,1> S = H*Pkp*~H+R;

    Matrix<1,1> S_inv = S;
    bool is_nonsingular = Invert(S_inv);
    if (!is_nonsingular) {
    return;
    }

    Matrix<7,1> K = Pkp*~H*S_inv;
    Xkp = Xkp+K*V;
    Pkp = Pkp-K*S*~K;
}

void OpticalEKF::updateGyroZ(float gyroz) {
    Matrix<1,7> H = {0,0,0,1,0,0,0};
    Matrix<1,1> R = {gyroZR};   

    //update step
    Matrix<1,1> y = {gyroz};
    Matrix<1,1> V = y-H*Xkp;
    Matrix<1,1> S = H*Pkp*~H+R;

    Matrix<1,1> S_inv = S;
    bool is_nonsingular = Invert(S_inv);
    if (!is_nonsingular) {
    return;
    }

    Matrix<7,1> K = Pkp*~H*S_inv;
    Xkp = Xkp+K*V;
    Pkp = Pkp-K*S*~K;
}

void OpticalEKF::updateAccelx(float accx) {
    Matrix<1,7> H = {0,0,0,0,0,1,0};
    Matrix<1,1> R = {accelXR};   

    //update step
    Matrix<1,1> y = {accx};
    Matrix<1,1> V = y-H*Xkp;
    Matrix<1,1> S = H*Pkp*~H+R;

    Matrix<1,1> S_inv = S;
    bool is_nonsingular = Invert(S_inv);
    if (!is_nonsingular) {
    return;
    }

    Matrix<7,1> K = Pkp*~H*S_inv;
    Xkp = Xkp+K*V;
    Pkp = Pkp-K*S*~K;
}