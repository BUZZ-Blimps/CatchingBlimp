#include "BangBang.h"

BangBang::BangBang(float newDeadBand, float newCenteringCom) {
    this->deadBand = newDeadBand;
    this->centeringCom = newCenteringCom;
}

float BangBang::calculate(float setPoint, float actual) {
    if (actual - setPoint > this->deadBand) {
        return -this->centeringCom;
    } else if (actual - setPoint < -this->deadBand) {
        return this->centeringCom;
    } else {
        return 0;
    }
}