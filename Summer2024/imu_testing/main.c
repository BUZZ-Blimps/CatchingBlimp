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
// #include "BasicLinearAlgebra.h"
// #include "ElementStorage.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <stdint.h>
// #include <linux/i2c-dev.h>
// #include <i2c/smbus.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>
// #include <wiringPiI2C.c>
// I2C definitions

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

static inline int i2c_smbus_access_bl (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args ;

  args.read_write = rw ;
  args.command    = command ;
  args.size       = size ;
  args.data       = data ;
  return ioctl (fd, I2C_SMBUS, &args) ;
}

int wiringPiI2CReadRegBlock (int fd, int reg, uint8_t *buff, int num_bytes)
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

int main (int argc, char *argv[]){
	const char *device;
	device = "/dev/i2c-3"; 

	int LIS3MDL = wiringPiI2CSetupInterface(device, LIS3MDL_ADDRESS);
	printf("i2c LIS3MDL fd: %d\n", LIS3MDL);
	// delay(100);

	int LSM6DSL = wiringPiI2CSetupInterface(device, LSM6DSL_ADDRESS);
	printf("i2c LSM6DSL fd: %d\n", LSM6DSL);

	int BM388 = wiringPiI2CSetupInterface(device, BM388_ADDRESS);
	printf("i2c LSM6DSL fd: %d\n", BM388);
	// delay(100);

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

	// printf("i2c read LSM6DSL: %d\n", wiringPiI2CRead(LSM6DSL));
	
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
	int out = wiringPiI2CReadRegBlock(BM388, NVM_PAR_T1_LSB, buff_calib, 21); 
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


	while(true) {
		out = wiringPiI2CReadRegBlock(LSM6DSL, LSM6DSL_OUT_X_L_XL, buff, 6); 
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

		printf("RAW: %d\n", out);
		printf("Test: %d\n", buff[0]);
		printf("AccYraw: %f\n", AccYraw);
		printf("AccXraw: %f\n", AccXraw);
		printf("AccZraw: %f\n\n", AccZraw);

		//Magnetometer Output
		out = wiringPiI2CReadRegBlock(LIS3MDL, 0x80 | LIS3MDL_OUT_X_L, buff, 6); 
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

		printf("RAW: %d\n", out);
		printf("Test: %d\n", buff[0]);
		printf("MagYraw: %f\n", MagYraw);
		printf("MagXraw: %f\n", MagXraw);
		printf("MagZraw: %f\n\n", MagZraw);

		//Gyroscope Output
		out = wiringPiI2CReadRegBlock(LSM6DSL, LSM6DSL_OUT_X_L_G, buff, 6); 
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

		printf("RAW: %d\n", out);
		printf("Test: %d\n", buff[0]);
		printf("gyr_rateYraw: %f\n", gyr_rateYraw);
		printf("gyr_rateXraw: %f\n", gyr_rateXraw);
		printf("gyr_rateZraw: %f\n\n", gyr_rateZraw);

		//Barometer and Temperature Sensor Output
		//Starts with the PRESS_XLSB_7_0 output register then will read the remainig five
		out = wiringPiI2CReadRegBlock(BM388, PRESS_XLSB_7_0, buff, 6); 
		if (out == -1) {
			printf("BM388 PRESS_XLSB_7_0 I2C not working\n");
		}

		// Last 3 bytes are the temperature XLSB, LSB, MSB
		float tempRaw = (int)(buff[3] | (buff[4] << 8) | (buff[5] << 16));
		printf("tempRaw: %f\n", tempRaw);
		// comp_temp = temp_compensation(tempRaw);
		//Serial.println(comp_temp); //Temperature in deg C

		// First 3 bytes are the pressure XLSB, LSB, MSB
		// Bit shift done by 256^3 = 16777216 and 16777216/3 = 5592405
		pressRaw = (int)(buff[0] | (buff[1] << 8) | (buff[2] << 16));
		printf("pressRaw: %f\n\n\n\n", pressRaw);


		delay(100);
	}

	



}
