#pragma once

#include "Arduino.h"
#include "Servo.h"
#include "EMAFilter.h"

class MotorControl {
    public:
        void update(double forward, double translation, double up, double yaw, double roll);
        double yawLeft = 0;
        double yawRight = 0;
        double upLeft = 0;
        double upRight = 0;
        double forwardLeft = 0;
        double forwardRight = 0;
};
