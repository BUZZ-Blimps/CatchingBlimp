#include "ICM_20948.h"


class IMU_Control {
    public:
    IMU_Control();
    void update();
    double getYawRate();
    double getPitchRate();
    double getRollRate();
    double getAccelX();
    double getAccelY();
    double getAccelZ();
    double getPitch();
    double getRoll();
    double getYaw();

    private:
    double yawRate;
    double rollRate;
    double pitchRate;

    double xAccel;
    double yAccel;
    double zAccel;

    double roll;
    double pitch;
    double yaw;
    
    ICM_20948_I2C myICM;

};
