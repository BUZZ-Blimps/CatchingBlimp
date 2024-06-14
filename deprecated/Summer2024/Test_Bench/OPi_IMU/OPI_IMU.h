/*
 BerryIMU_v3.h 
*/
#include <stdint.h>
#pragma once

class OPI_IMU
{
  public:
    void OPI_IMU_Setup();
    void IMU_read();
    void IMU_ROTATION(float rotation_angle);
    int LIS3MDL;
    int LSM6DSL;
    int BM388;
    //Maybe low pass filter applied depending on settings selected
    float AccXraw; 
    float AccYraw; 
    float AccZraw; 
    //Maybe high pass filter applied depending on settings selected
    float gyr_rateXraw; 
    float gyr_rateYraw; 
    float gyr_rateZraw;
    //magnetometer
    float MagXraw;
    float MagYraw;
    float MagZraw;
    //See cpp file for settings 
    float comp_temp;
    float comp_press;
    float ref_ground_press;
    float alt;
    float pressRaw;
    float tempRaw;

  private:
    float temp_compensation(float raw_temperature);
    float press_compensation(float raw_pressure, float comp_temp);
    int wiringPiI2CReadRegBlock (int fd, int reg, int num_bytes, uint8_t *buff);
    uint8_t buff[6];
    uint8_t buff_calib[21];
    int accRaw[3];
    int magRaw[3];
    int gyrRaw[3];
    bool ref_pressure_found;
    float PAR_T1;
    float PAR_T2;
    float PAR_T3;
    float PAR_P1;
    float PAR_P2;
    float PAR_P3;
    float PAR_P4;
    float PAR_P5;
    float PAR_P6;
    float PAR_P7;
    float PAR_P8;
    float PAR_P9;
    float PAR_P10;
    float PAR_P11;
};
