#pragma once

class BangBang {
    public:
    BangBang(float deadBand, float centeringCom);
    float calculate(float setPoint, float actural);

    private:
    float deadBand;
    float centeringCom;
};