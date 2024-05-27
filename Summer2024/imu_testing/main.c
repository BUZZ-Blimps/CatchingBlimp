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

#include <wiringPi.h>
#include <wiringPiI2C.h>

// typedef struct {
// 	unsigned int ccr;		// Capture/Compare Register (Duty Cycle)
// 	unsigned int arr;		// Auto-Reload Register (Period)
// 	unsigned int div;
// 	unsigned int div_stepping;
// } pwm_info;

// #define L	0
// #define R	2
// #define BL	5
// #define BR	8

// // Far ranges = [70,528]
// const int pwm_lower = 80;
// const int pwm_upper = 510;
// const int B_pwm_lower = 150;
// const int B_pwm_upper = 200;

// static pwm_info pwm_info_t;

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

	int accRaw[3];
	float AccXraw; 
    float AccYraw; 
    float AccZraw;

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

	while(true) {
		int out = wiringPiI2CReadReg16(LSM6DSL, LSM6DSL_OUT_X_L_XL); 
		accRaw[0] = out & 0xFFFF;
		accRaw[1] = out>>16 & 0xFFFF;
		accRaw[2] = out>>32 & 0xFFFF;

		if (accRaw[0] >= 32768) accRaw[0] = accRaw[0] - 65536;
		if (accRaw[1] >= 32768) accRaw[1] = accRaw[1] - 65536;
		if (accRaw[2] >= 32768) accRaw[2] = accRaw[2] - 65536;

		AccYraw = (accRaw[0] * 0.244) / 1000.0;
		AccXraw = -(accRaw[1] * 0.244) / 1000.0;
		AccZraw = (accRaw[2] * 0.244) / 1000.0;

		printf("RAW: %d\n", out);
		printf("AccYraw: %f\n", AccYraw);
		printf("AccXraw: %f\n", AccXraw);
		printf("AccZraw: %f\n", AccZraw);


		delay(100);
	}

	



}
