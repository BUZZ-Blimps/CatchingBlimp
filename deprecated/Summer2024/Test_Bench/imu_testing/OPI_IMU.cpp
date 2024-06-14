/*
Helpful commands:

To make a file executable: chmod 755 <filename>

To run the copy script: ./copyCodeToPi.sh <Orange Pi's IP>
- For Orange Pi number #, the IP address will be: 192.168.0.10#
- All laptops should also have the hostnames saved as opi#

To ssh into Orange Pi number #: ssh opi@opi#

To save an ssh password for Orange Pi number #: ssh-copy-id opi@opi#
*/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include "LSM6DSL.h"
#include "LIS3MDL.h"
#include "BM388.h"
#include <math.h>
#include "eigen3/Eigen/Dense"
// #include "BasicLinearAlgebra.h"
// #include "ElementStorage.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include "OPI_IMU.h"

#include <wiringPi.h>
#include <wiringPiI2C.h>

#define I2C_SLAVE	0x0703
#define I2C_SMBUS	0x0720	/* SMBus-level access */

#define I2C_SMBUS_READ	1
#define I2C_SMBUS_WRITE	0

// SMBus transaction types

#define I2C_SMBUS_QUICK		    0
#define I2C_SMBUS_BYTE		    1
#define I2C_SMBUS_BYTE_DATA	    2 
#define I2C_SMBUS_WORD_DATA	    3
#define I2C_SMBUS_PROC_CALL	    4
#define I2C_SMBUS_BLOCK_DATA	    5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7		/* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA    8

// SMBus messages

#define I2C_SMBUS_BLOCK_MAX	32	/* As specified in SMBus standard */	
#define I2C_SMBUS_I2C_BLOCK_MAX	32	/* Not specified but we use same structure */


// Structures used in the ioctl() calls
union i2c_smbus_data
{
  uint8_t  byte ;
  uint16_t word ;
  uint8_t  block [I2C_SMBUS_BLOCK_MAX + 2] ;	// block [0] is used for length + one more for PEC
} ;

struct i2c_smbus_ioctl_data
{
  char read_write ;
  uint8_t command ;
  int size ;
  union i2c_smbus_data *data ;
} ;

void OPI_IMU::OPI_IMU_Setup(){
    const char *device;
	device = "/dev/i2c-3"; 

    LIS3MDL = wiringPiI2CSetupInterface(device, LIS3MDL_ADDRESS);
    LSM6DSL = wiringPiI2CSetupInterface(device, LSM6DSL_ADDRESS);
    BM388 = wiringPiI2CSetupInterface(device, BM388_ADDRESS);

    wiringPiI2CWriteReg8(LSM6DSL, LSM6DSL_CTRL1_XL, 0b10011111);
	wiringPiI2CWriteReg8(LSM6DSL, LSM6DSL_CTRL8_XL, 0b11001000);
	wiringPiI2CWriteReg8(LSM6DSL, LSM6DSL_CTRL3_C, 0b01000100);
	wiringPiI2CWriteReg8(LSM6DSL, LSM6DSL_CTRL2_G, 0b10011100);

	wiringPiI2CWriteReg8(LIS3MDL, LIS3MDL_CTRL_REG1, 0b11011100);     // Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
	wiringPiI2CWriteReg8(LIS3MDL, LIS3MDL_CTRL_REG2, 0b00100000);     // +/- 8 gauss
	wiringPiI2CWriteReg8(LIS3MDL, LIS3MDL_CTRL_REG3, 0b00000000); 

	wiringPiI2CWriteReg8(BM388,PWR_CTRL, 0b00110011);     // Enables pressure sensor, Enables temperature sensor, Normal mode
	wiringPiI2CWriteReg8(BM388,OSR, 0b00001100);                 // x16 oversampling pressure measurement, x2 oversampling temp measurement
	wiringPiI2CWriteReg8(BM388,ODR, 0x03);                 // Output data rate 25Hz
	wiringPiI2CWriteReg8(BM388,CONFIG, 0b00000110);      // IIR filter coefficient of 63 

    //Loading the calibration values
	int out = wiringPiI2CReadRegBlock(BM388, NVM_PAR_T1_LSB, 21, buff_calib); 
	if (out == -1) {
		printf("BM388 I2C not working\n");
	}
	float NVM_PAR_T1_val = (uint16_t)(buff_calib[0] | (buff_calib[1] << 8));
	float NVM_PAR_T2_val = (uint16_t)(buff_calib[2] | (buff_calib[3] << 8));
	float NVM_PAR_T3_val = (int8_t)buff_calib[4];
	float NVM_PAR_P1_val = (int16_t)(buff_calib[5] | (buff_calib[6] << 8));
	float NVM_PAR_P2_val = (int16_t)(buff_calib[7] | (buff_calib[8] << 8));
	float NVM_PAR_P3_val = (int8_t)buff_calib[9];
	float NVM_PAR_P4_val = (int8_t)buff_calib[10];
	float NVM_PAR_P5_val = (uint16_t)(buff_calib[11] | (buff_calib[12] << 8));
	float NVM_PAR_P6_val = (uint16_t)(buff_calib[13] | (buff_calib[14] << 8));
	float NVM_PAR_P7_val = (int8_t)buff_calib[15];
	float NVM_PAR_P8_val = (int8_t)buff_calib[16];
	float NVM_PAR_P9_val = (int16_t)(buff_calib[17] | (buff_calib[18] << 8));
	float NVM_PAR_P10_val = (int8_t)buff_calib[19];
	float NVM_PAR_P11_val = (int8_t)buff_calib[20];

	//Floating point compensation
	PAR_T1 = NVM_PAR_T1_val / pow(2, -8);
	PAR_T2 = NVM_PAR_T2_val / pow(2, 30);
	PAR_T3 = NVM_PAR_T3_val / pow(2, 48);
	PAR_P1 = (NVM_PAR_P1_val - pow(2, 14)) / pow(2, 20);
	PAR_P2 = (NVM_PAR_P2_val - pow(2, 14)) / pow(2, 29);
	PAR_P3 = NVM_PAR_P3_val / pow(2, 32);
	PAR_P4 = NVM_PAR_P4_val / pow(2, 37);
	PAR_P5 = NVM_PAR_P5_val / pow(2, -3);
	PAR_P6 = NVM_PAR_P6_val / pow(2, 6);
	PAR_P7 = NVM_PAR_P7_val / pow(2, 8);
	PAR_P8 = NVM_PAR_P8_val / pow(2, 15);
	PAR_P9 = NVM_PAR_P9_val / pow(2, 48);
	PAR_P10 = NVM_PAR_P10_val / pow(2, 48);
	PAR_P11 = NVM_PAR_P11_val / pow(2, 65);


}

void OPI_IMU::OPI_IMU_read(){
    int out = wiringPiI2CReadRegBlock(LSM6DSL, LSM6DSL_OUT_X_L_XL, 6, buff); 
    if (out == -1) {
        printf("LSM6DSL I2C not working\n");
    }
    accRaw[0] = (int)(buff[0] | (buff[1] << 8));
    accRaw[1] = (int)(buff[2] | (buff[3] << 8));
    accRaw[2] = (int)(buff[4] | (buff[5] << 8));

    if (accRaw[0] >= 32768) accRaw[0] = accRaw[0] - 65536;
    if (accRaw[1] >= 32768) accRaw[1] = accRaw[1] - 65536;
    if (accRaw[2] >= 32768) accRaw[2] = accRaw[2] - 65536;

    AccYraw = (accRaw[0] * 0.244) / 1000.0;
    AccXraw = -(accRaw[1] * 0.244) / 1000.0;
    AccZraw = (accRaw[2] * 0.244) / 1000.0;


    //Magnetometer Output
    out = wiringPiI2CReadRegBlock(LIS3MDL, 0x80 | LIS3MDL_OUT_X_L, 6, buff); 
    if (out == -1) {
        printf("LIS3MDL I2C not working\n");
    }
    magRaw[0] = (int)(buff[0] | (buff[1] << 8));
    magRaw[1] = (int)(buff[2] | (buff[3] << 8));
    magRaw[2] = (int)(buff[4] | (buff[5] << 8));
    if (magRaw[0] >= 32768) magRaw[0] = magRaw[0] - 65536;
    if (magRaw[1] >= 32768) magRaw[1] = magRaw[1] - 65536;
    if (magRaw[2] >= 32768) magRaw[2] = magRaw[2] - 65536;

    //convert?
    MagYraw = magRaw[0];
    MagXraw = -magRaw[1];
    MagZraw = magRaw[2];


    //Gyroscope Output
    out = wiringPiI2CReadRegBlock(LSM6DSL, LSM6DSL_OUT_X_L_G, 6, buff); 
    if (out == -1) {
        printf("LSM6DSL_OUT_X_L_G I2C not working\n");
    }
    gyrRaw[0] = (int)(buff[0] | (buff[1] << 8));
    gyrRaw[1] = (int)(buff[2] | (buff[3] << 8));
    gyrRaw[2] = (int)(buff[4] | (buff[5] << 8));
    if (gyrRaw[0] >= 32768) gyrRaw[0] = gyrRaw[0] - 65536;
    if (gyrRaw[1] >= 32768) gyrRaw[1] = gyrRaw[1] - 65536;
    if (gyrRaw[2] >= 32768) gyrRaw[2] = gyrRaw[2] - 65536;

    //Convert Gyro raw to degrees per second updated (deg/s)
    gyr_rateYraw = (gyrRaw[0] * 70) / 1000.0;
    gyr_rateXraw = -(gyrRaw[1] * 70) / 1000.0;
    gyr_rateZraw = (gyrRaw[2] * 70) / 1000.0;
    

    //Barometer and Temperature Sensor Output
    //Starts with the PRESS_XLSB_7_0 output register then will read the remainig five
    out = wiringPiI2CReadRegBlock(BM388, PRESS_XLSB_7_0, 6, buff); 
    if (out == -1) {
        printf("BM388 PRESS_XLSB_7_0 I2C not working\n");
    }

    // Last 3 bytes are the temperature XLSB, LSB, MSB
    float tempRaw = (int)(buff[3] | (buff[4] << 8) | (buff[5] << 16));
    comp_temp = temp_compensation(tempRaw);
    // comp_temp = temp_compensation(tempRaw);
    //Serial.println(comp_temp); //Temperature in deg C

    // First 3 bytes are the pressure XLSB, LSB, MSB
    // Bit shift done by 256^3 = 16777216 and 16777216/3 = 5592405
    pressRaw = (int)(buff[0] | (buff[1] << 8) | (buff[2] << 16));
    comp_press = press_compensation(pressRaw, comp_temp);
    // Serial.println(comp_press); //Pressure in Pa

    //Altitude (in meters)
    //https://www.circuitbasics.com/set-bmp180-barometric-pressure-sensor-arduino/
    //Sets the reference pressure (therefore setting the reference height)
    if(ref_pressure_found){
        ref_ground_press = comp_press;
        ref_pressure_found = false;
    }
    alt = 44330 * (1 - pow((comp_press / ref_ground_press), (1 / 5.255))); //In meters

    // alt = comp_press;  // plus something from base station

    //---------------------------------------------------------------------------------------
}

// void OPI_IMU::OPI_IMU_ROTATION(float rotation_angle){  //current: 180 degrees z axis rotation
//     Eigen::Matrix3f Rz;
//     Rz <<   cosf(rotation_angle/180*M_PI),-sinf(rotation_angle/180*M_PI),0,
//             sinf(rotation_angle/180*M_PI),cosf(rotation_angle/180*M_PI),0,
//             0,0,1; // 3X3 Matrix

//     Eigen::MatrixXd gyr_rate(3, 1);
//     gyr_rate << gyr_rateXraw,
//                 gyr_rateYraw,
//                 gyr_rateZraw; 

//     Eigen::MatrixXd Acc_raw(3, 1);
//     Acc_raw <<  AccXraw,
//                 AccYraw,
//                 AccZraw; 

//     Eigen::MatrixXd corrected_gyr_rate(3, 1);
//     corrected_gyr_rate = Rz*gyr_rate;

//     Eigen::MatrixXd corrected_Acc_raw(3, 1);
//     corrected_Acc_raw = Rz*Acc_raw;

//     this->AccXraw = corrected_Acc_raw(0);
//     this->AccYraw = corrected_Acc_raw(1);
//     this->AccZraw = corrected_Acc_raw(2);

//     this->gyr_rateXraw = corrected_gyr_rate(0);
//     this->gyr_rateYraw = corrected_gyr_rate(1);
//     this->gyr_rateZraw = corrected_gyr_rate(2);
// }


float OPI_IMU::temp_compensation(float raw_temperature) {
  float partial_data1 = raw_temperature - PAR_T1;
  float partial_data2 = partial_data1 * PAR_T2;
  float comp_temp = partial_data2 + (partial_data1 * partial_data1) * PAR_T3;
  return comp_temp;
}

float OPI_IMU::press_compensation(float raw_pressure, float comp_temp) {
  float partial_data1 = PAR_P6 * comp_temp;
  float partial_data2 = PAR_P7 * (comp_temp * comp_temp);
  float partial_data3 = PAR_P8 * (comp_temp * comp_temp * comp_temp);
  float partial_out1 = PAR_P5 + partial_data1 + partial_data2 + partial_data3;
  partial_data1 = PAR_P2 * comp_temp;
  partial_data2 = PAR_P3 * (comp_temp * comp_temp);
  partial_data3 = PAR_P4 * (comp_temp * comp_temp * comp_temp);
  float partial_out2 = raw_pressure * (PAR_P1 + partial_data1 + partial_data2 + partial_data3);
  partial_data1 = raw_pressure * raw_pressure;
  partial_data2 = PAR_P9 + PAR_P10 * comp_temp;
  partial_data3 = partial_data1 * partial_data2;
  float partial_data4 = partial_data3 + (raw_pressure * raw_pressure * raw_pressure) * PAR_P11;
  float comp_press = partial_out1 + partial_out2 + partial_data4;

  return comp_press;
}

static inline int i2c_smbus_access_bl (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args ;

  args.read_write = rw ;
  args.command    = command ;
  args.size       = size ;
  args.data       = data ;
  return ioctl (fd, I2C_SMBUS, &args) ;
}

int wiringPiI2CReadRegBlock (int fd, int reg, int num_bytes, uint8_t *buff)
{
  union i2c_smbus_data data;
  if (i2c_smbus_access_bl (fd, I2C_SMBUS_READ, reg, 6, &data))
    return -1 ;
  else
	for (int i = 0; i < num_bytes; i++) {
		buff[i] = data.block[i + 1];
	}
    return data.block[0]; //first element is length of block array
}