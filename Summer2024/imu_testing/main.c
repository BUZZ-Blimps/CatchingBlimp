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
	int LIS3MDL = wiringPiI2CSetup(LIS3MDL_ADDRESS);
	printf("i2c LIS3MDL: %d\n", LIS3MDL);
	// delay(100);

	int LSM6DSL = wiringPiI2CSetup(LSM6DSL_ADDRESS);
	printf("i2c LSM6DSL: %d\n", LSM6DSL);
	// delay(100);

	// printf("i2c read LSM6DSL: %d\n", wiringPiI2CRead(LSM6DSL));
	
	wiringPiI2CWriteReg8(LSM6DSL, LSM6DSL_CTRL1_XL, 0b10011111);
	wiringPiI2CWriteReg8(LSM6DSL, LSM6DSL_CTRL8_XL, 0b11001000);
	wiringPiI2CWriteReg8(LSM6DSL, LSM6DSL_CTRL3_C, 0b01000100);
	wiringPiI2CWriteReg8(LSM6DSL, LSM6DSL_CTRL2_G, 0b10011100);

	while(true) {
		printf("i2c LSM6DSL: %d\n", wiringPiI2CReadReg16(LSM6DSL, LSM6DSL_OUT_X_L_XL)); 
		delay(100);
	}

	



}
