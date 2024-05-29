#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <wiringPi.h>


#include "OPI_IMU.h"


int main(){
    OPI_IMU imu;
    imu.OPI_IMU_Setup();
    // while(1){
    //     imu.OPI_IMU_read();
    //     // imu.OPI_IMU_ROTATION(0); // Rotate IMU

    //     printf("AccYraw: %f\n", imu.AccYraw);
    //     printf("AccXraw: %f\n", imu.AccXraw);
    //     printf("AccZraw: %f\n\n", imu.AccZraw);

    //     printf("MagYraw: %f\n", imu.MagYraw);
    //     printf("MagXraw: %f\n", imu.MagXraw);
    //     printf("MagZraw: %f\n\n", imu.MagZraw);

    //     printf("gyr_rateYraw: %f\n", imu.gyr_rateYraw);
    //     printf("gyr_rateXraw: %f\n", imu.gyr_rateXraw);
    //     printf("gyr_rateZraw: %f\n\n", imu.gyr_rateZraw);

    //     printf("tempRaw: %f\n", imu.tempRaw);
    //     printf("pressRaw: %f\n\n\n\n", imu.pressRaw);
    //     delay(3000);
    // }
    return(1);
}